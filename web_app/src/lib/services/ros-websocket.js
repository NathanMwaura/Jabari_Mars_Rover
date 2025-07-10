import io from 'socket.io-client';
import ReconnectingWebSocket from 'reconnecting-websocket';
import { robotStatus, sensorData, systemStats, cameraState, lastCommand } from '../stores';

const ROS_BRIDGE_URL = 'ws://localhost:9090';

let socket;

export function initRosWebsocket() {
  socket = new ReconnectingWebSocket(ROS_BRIDGE_URL);

  socket.addEventListener('open', () => {
    console.log('Connected to ROS Bridge');
    robotStatus.update(status => ({ ...status, isConnected: true }));
  });

  socket.addEventListener('close', () => {
    console.log('Disconnected from ROS Bridge');
    robotStatus.update(status => ({ ...status, isConnected: false }));
  });

  socket.addEventListener('message', (event) => {
    const message = JSON.parse(event.data);
    handleRosMessage(message);
  });

  socket.addEventListener('error', (error) => {
    console.error('ROS Bridge connection error:', error);
  });
}

function handleRosMessage(message) {
  const { op, topic, msg } = message;

  if (op === 'publish') {
    switch (topic) {
      case '/system_status':
        systemStats.set(msg);
        break;
      case '/sensor_data':
        sensorData.set(msg);
        break;
      case '/camera_state':
        cameraState.set(msg);
        break;
      default:
        break;
    }
  }
}

export function sendRosMessage(topic, message) {
  if (socket && socket.readyState === ReconnectingWebSocket.OPEN) {
    const rosMessage = {
      op: 'publish',
      topic: topic,
      msg: message,
    };
    socket.send(JSON.stringify(rosMessage));
    lastCommand.set(JSON.stringify(message));
  } else {
    console.error('ROS Bridge not connected. Cannot send message.');
  }
}