import { writable } from 'svelte/store';

export const robotStatus = writable({
  isConnected: false,
  isPaused: false,
  controlMode: 'manual',
});

export const sensorData = writable({
  temperature: 36.5,
  battery: 85,
  signal: -45,
  memory: 67,
  cpu: 34,
  distance: 1.2
});

export const systemStats = writable({
  uptime: { hours: 2, minutes: 34 },
  latency: 12,
  messages: 247,
  disk: 45
});

export const cameraState = writable({
  pan: 0,
  tilt: 0,
  quality: 'medium',
  isRecording: false,
});

export const lastCommand = writable('');