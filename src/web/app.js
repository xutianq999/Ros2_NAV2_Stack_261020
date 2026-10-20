// ============================================
// Application State（适配本项目: Livox MID360 + Super-LIO / FAST-LIO + Nav2）
// ============================================
const state = {
    ros: null,
    viewer3d: null,
    tfClient: null,
    pointCloudClient: null,
    imuSubscriber: null,
    cmdVelPublisher: null,
    currentVelocity: { linear: 0, angular: 0 },
    connected: false,
    robotControlBound: false,
    config: {
        rosbridgeUrl: 'ws://localhost:9090',
        fixedFrame: 'odom',                 // /registered_scan 在 odom 系（旧链路 lio_interface / 新桥接 external 模式）
        pointCloudTopic: '/registered_scan',
        colorMap: 'auto',                   // auto: 按强度场上色（LIO 点云含 intensity）
        imuTopic: '/livox/imu'              // MID360 内置 IMU（LIO 输入源）
    }
};

// 强度上色的动态归一化范围（随数据自适应）
const intensityRange = { min: 0, max: 200 };

// ============================================
// DOM Elements
// ============================================
const elements = {
    // Connection
    statusIndicator: document.getElementById('status-indicator'),
    rosbridgeInput: document.getElementById('rosbridge-url'),
    connectBtn: document.getElementById('connect-btn'),
    workspaceFrame: document.getElementById('workspace-frame'),

    // Configuration
    fixedFrameInput: document.getElementById('fixed-frame'),
    pointCloudInput: document.getElementById('pointcloud-topic'),
    colorMapSelect: document.getElementById('color-map'),
    applyColorBtn: document.getElementById('apply-color-btn'),

    // Robot Control
    linearSpeedSlider: document.getElementById('linear-speed'),
    angularSpeedSlider: document.getElementById('angular-speed'),
    linearSpeedValue: document.getElementById('linear-speed-value'),
    angularSpeedValue: document.getElementById('angular-speed-value'),
    stopBtn: document.getElementById('stop-btn'),
    currentLinear: document.getElementById('current-linear'),
    currentAngular: document.getElementById('current-angular'),

    // IMU Data
    imuRoll: document.getElementById('imu-roll'),
    imuPitch: document.getElementById('imu-pitch'),
    imuYaw: document.getElementById('imu-yaw'),
    imuAccelX: document.getElementById('imu-accel-x'),
    imuAccelY: document.getElementById('imu-accel-y'),
    imuAccelZ: document.getElementById('imu-accel-z'),
    imuGyroX: document.getElementById('imu-gyro-x'),
    imuGyroY: document.getElementById('imu-gyro-y'),
    imuGyroZ: document.getElementById('imu-gyro-z')
};

// ============================================
// Initialization
// ============================================
function init() {
    console.log('Initializing Robot Center (ROS2)...');

    if (typeof ROSLIB === 'undefined') {
        console.error('ROSLIB 未加载，请检查 lib/ 目录或网络');
        return;
    }

    // 自动替换默认 WebSocket 地址里的 hostname
    if (elements.rosbridgeInput.value.includes('localhost')) {
        elements.rosbridgeInput.value = `ws://${window.location.hostname}:9090`;
    }

    updateConfigFromUI();

    elements.connectBtn.addEventListener('click', toggleConnection);
    elements.fixedFrameInput.addEventListener('change', updateConfigFromUI);
    elements.pointCloudInput.addEventListener('change', updateConfigFromUI);
    elements.applyColorBtn.addEventListener('click', applyColorSettings);

    console.log('Initialization complete');
}

function updateConfigFromUI() {
    state.config.rosbridgeUrl = elements.rosbridgeInput.value;
    state.config.fixedFrame = elements.fixedFrameInput.value.trim();
    state.config.pointCloudTopic = elements.pointCloudInput.value.trim();
    state.config.colorMap = elements.colorMapSelect.value;
    if (elements.workspaceFrame) elements.workspaceFrame.textContent = state.config.fixedFrame || 'odom';
}

function applyColorSettings() {
    if (!state.connected) {
        console.warn('Not connected to ROS. Connect first.');
        return;
    }
    updateConfigFromUI();
    init3DViewer();
}

// ============================================
// Connection Management
// ============================================
function toggleConnection() {
    if (state.connected) disconnect();
    else connect();
}

function connect() {
    updateConfigFromUI();
    console.log('Connecting to:', state.config.rosbridgeUrl);

    state.ros = new ROSLIB.Ros({ url: state.config.rosbridgeUrl });

    state.ros.on('connection', () => {
        console.log('Connected to ROSBridge');
        state.connected = true;
        updateConnectionStatus(true);
        elements.connectBtn.textContent = 'Disconnect';

        init3DViewer();
        initIMUSubscriber();
        initRobotControl();
    });

    state.ros.on('error', (error) => {
        console.error('Connection error:', error);
        updateConnectionStatus(false);
    });

    state.ros.on('close', () => {
        console.log('Connection closed');
        state.connected = false;
        updateConnectionStatus(false);
        elements.connectBtn.textContent = 'Connect';
        cleanup();
    });
}

function disconnect() {
    if (state.ros) state.ros.close();
}

function updateConnectionStatus(connected) {
    if (connected) {
        elements.statusIndicator.classList.add('connected');
        elements.statusIndicator.querySelector('.status-text').textContent = 'Connected';
    } else {
        elements.statusIndicator.classList.remove('connected');
        elements.statusIndicator.querySelector('.status-text').textContent = 'Disconnected';
    }
}

function cleanup() {
    if (state.imuSubscriber) state.imuSubscriber.unsubscribe();
    if (state.pointCloudClient) state.pointCloudClient.unsubscribe();

    const container = document.getElementById('viewer-3d');
    if (container) container.innerHTML = '';

    state.viewer3d = null;
    state.tfClient = null;
    state.pointCloudClient = null;
}

// ============================================
// 3D Visualization
// ============================================
// 点云默认在 odom 系下发布（与 fixed frame 一致），此时不需要 TF 变换。
// 用一个恒等透传的假 tfClient 代替 ROSLIB.TFClient，
// 避免依赖机器人端的 tf2_web_republisher。
function makeIdentityTfClient() {
    const identity = { position: { x: 0, y: 0, z: 0 }, orientation: { x: 0, y: 0, z: 0, w: 1 } };
    return {
        fixedFrame: state.config.fixedFrame,
        subscribe: (frame, cb) => cb(identity),
        unsubscribe: () => { }
    };
}

function intensityColormap(x) {
    const t = Math.max(0, Math.min(1, (x - intensityRange.min) / (intensityRange.max - intensityRange.min)));
    return new THREE.Color().setHSL(0.7 - 0.7 * t, 0.9, 0.5);   // 蓝 -> 绿 -> 红
}

function init3DViewer() {
    console.log('Initializing 3D viewer...');

    const container = document.getElementById('viewer-3d');
    container.innerHTML = '';

    if (!isWebGLAvailable()) {
        showWebGLError(container);
        return;
    }

    try {
        state.viewer3d = new ROS3D.Viewer({
            divID: 'viewer-3d',
            width: container.clientWidth,
            height: container.clientHeight,
            antialias: true,
            background: '#0a0e14'
        });
    } catch (e) {
        console.error('Failed to create WebGL context:', e);
        showWebGLError(container, e.message);
        return;
    }

    const grid = new ROS3D.Grid({ color: '#2d3640', cellSize: 1, num_cells: 30 });
    state.viewer3d.addObject(grid);

    // fixed frame 与点云坐标系一致（默认 odom == odom）时用恒等透传；
    // 改成其他 fixed frame 时才需要真正的 TFClient（需机器人端 tf2_web_republisher）。
    if (state.config.fixedFrame === 'odom') {
        state.tfClient = makeIdentityTfClient();
    } else {
        console.warn('Fixed frame 与点云坐标系不同，TFClient 需要 tf2_web_republisher 支持');
        state.tfClient = new ROSLIB.TFClient({
            ros: state.ros,
            angularThres: 0.01,
            transThres: 0.01,
            rate: 10.0,
            fixedFrame: state.config.fixedFrame
        });
    }

    // auto: 按 intensity 场上色；solid: 纯色
    let materialConfig = { size: 2, sizeAttenuation: false };
    let colorsrc;
    if (state.config.colorMap === 'green') {
        materialConfig.color = 0x00ff00;
    } else if (state.config.colorMap === 'cyan') {
        materialConfig.color = 0x00ffff;
    } else if (state.config.colorMap === 'white') {
        materialConfig.color = 0xffffff;
    } else {
        // auto: LIO 点云带 intensity 字段
        materialConfig.color = 0xffffff;
        colorsrc = 'intensity';
    }

    state.pointCloudClient = new ROS3D.PointCloud2({
        ros: state.ros,
        tfClient: state.tfClient,
        topic: state.config.pointCloudTopic,
        rootObject: state.viewer3d.scene,
        max_pts: 48000,
        throttle_rate: 500,          // 2Hz 足够观察，避免 JSON 序列化洪泛
        colorsrc: colorsrc,
        colormap: colorsrc === 'intensity' ? intensityColormap : undefined,
        material: materialConfig
    });

    // resize 只绑定一次，避免多次重连后叠加监听器
    if (!state.resizeBound) {
        state.resizeBound = true;
        window.addEventListener('resize', () => {
            if (state.viewer3d) {
                state.viewer3d.resize(container.clientWidth, container.clientHeight);
            }
        });
    }

    console.log('3D viewer initialized');
}

// ============================================
// IMU Data Subscriber（/livox/imu, sensor_msgs/Imu）
// ============================================
function initIMUSubscriber() {
    console.log('Subscribing to IMU:', state.config.imuTopic);

    state.imuSubscriber = new ROSLIB.Topic({
        ros: state.ros,
        name: state.config.imuTopic,
        messageType: 'sensor_msgs/Imu',
        throttle_rate: 100
    });

    state.imuSubscriber.subscribe((message) => {
        const euler = quaternionToEuler(
            message.orientation.x, message.orientation.y,
            message.orientation.z, message.orientation.w);

        elements.imuRoll.textContent = euler.roll.toFixed(2) + '°';
        elements.imuPitch.textContent = euler.pitch.toFixed(2) + '°';
        elements.imuYaw.textContent = euler.yaw.toFixed(2) + '°';

        elements.imuAccelX.textContent = message.linear_acceleration.x.toFixed(3) + ' m/s²';
        elements.imuAccelY.textContent = message.linear_acceleration.y.toFixed(3) + ' m/s²';
        elements.imuAccelZ.textContent = message.linear_acceleration.z.toFixed(3) + ' m/s²';

        elements.imuGyroX.textContent = message.angular_velocity.x.toFixed(3) + ' rad/s';
        elements.imuGyroY.textContent = message.angular_velocity.y.toFixed(3) + ' rad/s';
        elements.imuGyroZ.textContent = message.angular_velocity.z.toFixed(3) + ' rad/s';
    });
}

// ============================================
// Utility Functions
// ============================================
function quaternionToEuler(x, y, z, w) {
    const sinr_cosp = 2 * (w * x + y * z);
    const cosr_cosp = 1 - 2 * (x * x + y * y);
    const roll = Math.atan2(sinr_cosp, cosr_cosp);

    const sinp = 2 * (w * y - z * x);
    const pitch = Math.abs(sinp) >= 1
        ? Math.sign(sinp) * Math.PI / 2
        : Math.asin(sinp);

    const siny_cosp = 2 * (w * z + x * y);
    const cosy_cosp = 1 - 2 * (y * y + z * z);
    const yaw = Math.atan2(siny_cosp, cosy_cosp);

    return {
        roll: roll * 180 / Math.PI,
        pitch: pitch * 180 / Math.PI,
        yaw: yaw * 180 / Math.PI
    };
}

function isWebGLAvailable() {
    try {
        const canvas = document.createElement('canvas');
        return !!(window.WebGLRenderingContext && (canvas.getContext('webgl') || canvas.getContext('experimental-webgl')));
    } catch (e) {
        return false;
    }
}

function showWebGLError(container, details = '') {
    container.innerHTML = `
        <div class="webgl-error-overlay">
            <div class="webgl-error-content">
                <div class="webgl-error-icon">⚠️</div>
                <h3>WebGL 初始化失败</h3>
                <p>无法创建 WebGL 上下文，这通常是由于浏览器禁用了硬件加速或显卡驱动不匹配导致的。</p>
                ${details ? `<p class="error-details">错误详情: ${details}</p>` : ''}
                <div class="troubleshooting-steps">
                    <h4>解决方法:</h4>
                    <ul>
                        <li>确保浏览器设置中开启了<strong>“使用硬件加速”</strong>。</li>
                        <li>在浏览器地址栏输入 <code>about:flags</code> 或 <code>chrome://flags</code>，搜索并开启 <strong>“Override software rendering list”</strong>。</li>
                        <li>如果以上无效，尝试在启动浏览器时加入参数：<code>--enable-unsafe-swiftshader</code> （风险自担）。</li>
                        <li>检查显卡驱动是否已更新到最新版本。</li>
                    </ul>
                </div>
            </div>
        </div>
    `;
}

// ============================================
// Robot Control（手动遥控，发布 /cmd_vel_teleop）
// 注意：安全层启用后，/cmd_vel 由 cmd_vel_deadman 独占（它直连底盘），
// 遥控走 /cmd_vel_teleop，由 twist_mux 按优先级放行（遥控优先级高于自动驾驶）。
// 详见 src/nav2_safety/README.md 与 docs/ENABLING.md。
// ============================================
function initRobotControl() {
    state.cmdVelPublisher = new ROSLIB.Topic({
        ros: state.ros,
        name: '/cmd_vel_teleop',
        messageType: 'geometry_msgs/Twist'
    });

    console.log('Robot control initialized. Publishing to /cmd_vel_teleop');

    if (state.robotControlBound) return;
    state.robotControlBound = true;

    elements.linearSpeedSlider.addEventListener('input', (e) => {
        elements.linearSpeedValue.textContent = parseFloat(e.target.value).toFixed(2);
    });

    elements.angularSpeedSlider.addEventListener('input', (e) => {
        elements.angularSpeedValue.textContent = parseFloat(e.target.value).toFixed(2);
    });

    const controlButtons = document.querySelectorAll('.control-btn[data-direction]');
    controlButtons.forEach(btn => {
        btn.addEventListener('mousedown', () => handleDirectionPress(btn.dataset.direction));
        btn.addEventListener('mouseup', stopRobot);
        btn.addEventListener('mouseleave', stopRobot);
        btn.addEventListener('touchstart', (e) => {
            e.preventDefault();
            handleDirectionPress(btn.dataset.direction);
        });
        btn.addEventListener('touchend', (e) => {
            e.preventDefault();
            stopRobot();
        });
    });

    elements.stopBtn.addEventListener('click', stopRobot);

    document.addEventListener('keydown', handleKeyDown);
    document.addEventListener('keyup', handleKeyUp);
    // 切换窗口时按键的 keyup 会丢失，立即停车防止机器人持续移动
    window.addEventListener('blur', stopRobot);
}

function handleDirectionPress(direction) {
    const linearSpeed = parseFloat(elements.linearSpeedSlider.value);
    const angularSpeed = parseFloat(elements.angularSpeedSlider.value);

    const directionMap = {
        'forward': { linear: linearSpeed, angular: 0 },
        'backward': { linear: -linearSpeed, angular: 0 },
        'left': { linear: 0, angular: angularSpeed },
        'right': { linear: 0, angular: -angularSpeed },
        'forward-left': { linear: linearSpeed, angular: angularSpeed },
        'forward-right': { linear: linearSpeed, angular: -angularSpeed },
        'backward-left': { linear: -linearSpeed, angular: angularSpeed },
        'backward-right': { linear: -linearSpeed, angular: -angularSpeed }
    };

    const velocity = directionMap[direction];
    if (velocity) {
        publishVelocity(velocity.linear, velocity.angular);
    }
}

let activeKeys = new Set();

// 焦点在输入控件时不响应键盘遥控（否则在地址栏输入 ws://... 会触发 w/s 移动）
function isTypingTarget(e) {
    const t = e.target;
    return t && (t.tagName === 'INPUT' || t.tagName === 'SELECT' || t.tagName === 'TEXTAREA');
}

function handleKeyDown(e) {
    if (isTypingTarget(e)) return;

    if (['ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight'].includes(e.key)) {
        e.preventDefault();
    }

    activeKeys.add(e.key);
    updateVelocityFromKeys();
}

function handleKeyUp(e) {
    if (isTypingTarget(e)) return;

    activeKeys.delete(e.key);
    updateVelocityFromKeys();
}

function updateVelocityFromKeys() {
    const linearSpeed = parseFloat(elements.linearSpeedSlider.value);
    const angularSpeed = parseFloat(elements.angularSpeedSlider.value);

    let linear = 0;
    let angular = 0;

    if (activeKeys.has('w') || activeKeys.has('W') || activeKeys.has('ArrowUp')) {
        linear += linearSpeed;
    }
    if (activeKeys.has('s') || activeKeys.has('S') || activeKeys.has('ArrowDown')) {
        linear -= linearSpeed;
    }
    if (activeKeys.has('a') || activeKeys.has('A') || activeKeys.has('ArrowLeft')) {
        angular += angularSpeed;
    }
    if (activeKeys.has('d') || activeKeys.has('D') || activeKeys.has('ArrowRight')) {
        angular -= angularSpeed;
    }

    if (activeKeys.has(' ')) {
        linear = 0;
        angular = 0;
    }

    publishVelocity(linear, angular);
}

function publishVelocity(linear, angular) {
    if (!state.connected || !state.cmdVelPublisher) {
        console.warn('Not connected to ROS. Cannot publish velocity.');
        return;
    }

    const twist = new ROSLIB.Message({
        linear: { x: linear, y: 0.0, z: 0.0 },
        angular: { x: 0.0, y: 0.0, z: angular }
    });

    state.cmdVelPublisher.publish(twist);

    state.currentVelocity.linear = linear;
    state.currentVelocity.angular = angular;
    elements.currentLinear.textContent = linear.toFixed(2);
    elements.currentAngular.textContent = angular.toFixed(2);
}

function stopRobot() {
    publishVelocity(0, 0);
    activeKeys.clear();
}

// ============================================
// Start Application
// ============================================
init();
