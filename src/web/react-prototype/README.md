# Robot Center React prototype

This isolated prototype explores a React/Vite component architecture without changing the existing static pages.

## Run

```bash
npm install
npm run dev
```

The prototype uses mock telemetry and a lightweight canvas point-cloud visualization. ROS2 wiring is intentionally not connected yet.

## Applied React Best Practices

- `memo` for repeated status cells
- stable callbacks with `useCallback`
- derived display values with `useMemo`
- one global keyboard listener effect with cleanup
- canvas animation isolated in a leaf component with cleanup
- no inline component definitions inside render loops
