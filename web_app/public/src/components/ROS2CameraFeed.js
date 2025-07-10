class ROS2CameraFeed {
    constructor(containerId) {
        this.container = document.getElementById(containerId);
        this.isStreaming = false;
        this.isRecording = false;
        this.currentQuality = 'medium';
        this.isFullscreen = false;
        this.videoElement = null;
        this.ros = null;
        this.imageSubscriber = null;
        this.cameraInfoSubscriber = null;
        this.streamUrl = null;
        this.lastFrameTime = null;
        this.frameCount = 0;
        this.recordingStartTime = null;
        
        this.init();
    }

    init() {
        this.createCameraInterface();
        this.initializeROS2();
        this.bindEvents();
    }

    // Notification system for user feedback
    showNotification(message, type = 'info', duration = 3000) {
        // Create notification container if it doesn't exist
        let notificationContainer = document.querySelector('.notification-container');
        if (!notificationContainer) {
            notificationContainer = document.createElement('div');
            notificationContainer.className = 'notification-container';
            notificationContainer.style.cssText = `
                position: fixed;
                top: 20px;
                right: 20px;
                z-index: 1000;
                display: flex;
                flex-direction: column;
                gap: 10px;
            `;
            document.body.appendChild(notificationContainer);
        }

        // Create notification
        const notification = document.createElement('div');
        notification.className = `notification notification-${type}`;
        notification.style.cssText = `
            background: ${type === 'success' ? '#4CAF50' : 
                        type === 'error' ? '#f44336' : 
                        type === 'warning' ? '#ff9800' : '#2196F3'};
            color: white;
            padding: 12px 20px;
            border-radius: 4px;
            box-shadow: 0 2px 10px rgba(0,0,0,0.2);
            animation: slideIn 0.3s ease-out;
            max-width: 300px;
            word-wrap: break-word;
        `;
        notification.textContent = message;

        // Add CSS animation if not already present
        if (!document.querySelector('#notification-styles')) {
            const style = document.createElement('style');
            style.id = 'notification-styles';
            style.textContent = `
                @keyframes slideIn {
                    from { transform: translateX(100%); opacity: 0; }
                    to { transform: translateX(0); opacity: 1; }
                }
                @keyframes slideOut {
                    from { transform: translateX(0); opacity: 1; }
                    to { transform: translateX(100%); opacity: 0; }
                }
            `;
            document.head.appendChild(style);
        }

        notificationContainer.appendChild(notification);

        // Auto-remove notification
        setTimeout(() => {
            notification.style.animation = 'slideOut 0.3s ease-out';
            setTimeout(() => {
                if (notification.parentNode) {
                    notification.parentNode.removeChild(notification);
                }
            }, 300);
        }, duration);
    }

    // Create the complete camera interface with activity log
    createCameraInterface() {
        this.container.innerHTML = `
            <div class="camera-container">
                <div class="camera-header">
                    <h3>ROS2 PiCamera Feed</h3>
                    <div class="camera-controls">
                        <select id="qualitySelect" class="control-select">
                            <option value="low">Low (480p)</option>
                            <option value="medium" selected>Medium (720p)</option>
                            <option value="high">High (1080p)</option>
                        </select>
                        <button id="connectBtn" class="control-btn connect-btn">
                            <i class="icon-connect"></i> Connect
                        </button>
                        <button id="recordBtn" class="control-btn record-btn" disabled>
                            <i class="icon-record"></i> Record
                        </button>
                        <button id="fullscreenBtn" class="control-btn">
                            <i class="icon-fullscreen"></i> Fullscreen
                        </button>
                    </div>
                </div>
                
                <div class="camera-viewport" id="cameraViewport">
                    <video id="videoElement" width="640" height="480" autoplay muted></video>
                    <img id="imageElement" width="640" height="480" style="display:none;" />
                    <div class="camera-overlay">
                        <div class="status-indicator ${this.isStreaming ? 'streaming' : 'offline'}">
                            <span class="status-dot"></span>
                            <span class="status-text">${this.isStreaming ? 'LIVE' : 'OFFLINE'}</span>
                        </div>
                        <div class="camera-info">
                            <div class="info-item">FPS: <span id="fpsCounter">0</span></div>
                            <div class="info-item">Quality: <span id="qualityDisplay">720p</span></div>
                            <div class="info-item">Latency: <span id="latencyDisplay">--ms</span></div>
                            <div class="info-item">ROS2: <span id="rosStatus">Disconnected</span></div>
                        </div>
                    </div>
                </div>
                
                <div class="camera-footer">
                    <div class="connection-status">
                        <span>WebSocket: <span id="wsStatus">Disconnected</span></span>
                        <span>Topic: <span id="topicStatus">/camera/image_raw</span></span>
                    </div>
                    <div class="recording-controls" style="display: none;">
                        <span class="recording-indicator">
                            <span class="recording-dot"></span>
                            Recording: <span id="recordingTime">00:00</span>
                        </span>
                        <button id="stopRecordBtn" class="control-btn stop-btn">Stop Recording</button>
                    </div>
                </div>
                
                <!-- Activity Log Panel -->
                <div class="activity-log" id="activityLog" style="display: none;">
                    <div class="log-header">
                        <h4>Activity Log</h4>
                        <button id="toggleLogBtn" class="control-btn">Hide Log</button>
                        <button id="clearLogBtn" class="control-btn">Clear</button>
                    </div>
                    <div class="log-content" id="logContent"></div>
                </div>
                
                <div class="log-toggle-container">
                    <button id="showLogBtn" class="control-btn">Show Activity Log</button>
                </div>
            </div>
        `;
    }

    // Enhanced logging method with console and UI logging
    logActivity(message, type = 'info') {
        const timestamp = new Date().toLocaleTimeString();
        const logContent = document.getElementById('logContent');
        
        if (logContent) {
            const logEntry = document.createElement('div');
            logEntry.className = `log-entry log-${type}`;
            logEntry.innerHTML = `
                <span class="log-time">[${timestamp}]</span>
                <span class="log-message">${message}</span>
            `;
            
            logContent.appendChild(logEntry);
            logContent.scrollTop = logContent.scrollHeight;
            
            // Limit log entries to prevent memory issues
            if (logContent.children.length > 100) {
                logContent.removeChild(logContent.firstChild);
            }
        }
        
        // Also log to console for debugging
        console.log(`[${timestamp}] ${message}`);
    }

    // Enhanced ROS2 initialization with comprehensive feedback
    async initializeROS2() {
        try {
            this.logActivity('Initializing ROS2 connection...', 'info');
            this.showNotification('Connecting to ROS2...', 'info');
            
            // Initialize rosbridge connection
            this.ros = new ROSLIB.Ros({
                url: 'ws://localhost:9090'  // Default rosbridge websocket
            });

            this.ros.on('connection', () => {
                this.logActivity('Successfully connected to ROS2 via rosbridge', 'success');
                this.showNotification('Connected to ROS2!', 'success');
                document.getElementById('rosStatus').textContent = 'Connected';
                document.getElementById('wsStatus').textContent = 'Connected';
                document.getElementById('connectBtn').disabled = false;
                this.setupROS2Subscribers();
            });

            this.ros.on('error', (error) => {
                this.logActivity(`ROS2 connection error: ${error}`, 'error');
                this.showNotification('ROS2 connection failed!', 'error');
                document.getElementById('rosStatus').textContent = 'Error';
                document.getElementById('wsStatus').textContent = 'Error';
            });

            this.ros.on('close', () => {
                this.logActivity('ROS2 connection closed', 'warning');
                this.showNotification('ROS2 connection lost', 'warning');
                document.getElementById('rosStatus').textContent = 'Disconnected';
                document.getElementById('wsStatus').textContent = 'Disconnected';
                this.isStreaming = false;
                this.updateStatus();
            });

        } catch (error) {
            this.logActivity(`Failed to initialize ROS2: ${error.message}`, 'error');
            this.showNotification('ROS2 initialization failed!', 'error');
            console.error('Failed to initialize ROS2:', error);
        }
    }

    // Setup ROS2 subscribers for image and camera info
    setupROS2Subscribers() {
        this.logActivity('Setting up ROS2 subscribers...', 'info');
        
        // Subscribe to compressed image topic
        this.imageSubscriber = new ROSLIB.Topic({
            ros: this.ros,
            name: '/camera/image_raw/compressed',
            messageType: 'sensor_msgs/msg/CompressedImage'
        });

        // Subscribe to camera info topic
        this.cameraInfoSubscriber = new ROSLIB.Topic({
            ros: this.ros,
            name: '/camera/camera_info',
            messageType: 'sensor_msgs/msg/CameraInfo'
        });

        // Handle incoming images
        this.imageSubscriber.subscribe((message) => {
            this.handleImageMessage(message);
        });

        // Handle camera info
        this.cameraInfoSubscriber.subscribe((message) => {
            this.handleCameraInfo(message);
        });

        this.logActivity('ROS2 subscribers configured successfully', 'success');
    }

    // Handle incoming image messages
    handleImageMessage(message) {
        // Convert base64 encoded image to blob URL
        const imageData = 'data:image/jpeg;base64,' + message.data;
        const imageElement = document.getElementById('imageElement');
        imageElement.src = imageData;
        
        // Update FPS counter
        this.updateFPS();
        
        // Calculate and display latency
        const currentTime = Date.now();
        const messageTime = message.header.stamp.sec * 1000 + message.header.stamp.nanosec / 1000000;
        const latency = currentTime - messageTime;
        document.getElementById('latencyDisplay').textContent = `${latency.toFixed(0)}ms`;
    }

    // Handle camera info messages
    handleCameraInfo(message) {
        // Update camera information display
        this.logActivity('Camera info received', 'info');
        console.log('Camera info received:', message);
        // You can extract and display camera parameters here
    }

    // Enhanced streaming controls with comprehensive feedback
    startStreaming() {
        if (!this.ros || !this.ros.isConnected) {
            this.logActivity('Cannot start streaming: ROS2 not connected', 'error');
            this.showNotification('Please connect to ROS2 first', 'error');
            return;
        }

        this.isStreaming = true;
        this.logActivity('Camera streaming started', 'success');
        this.showNotification('Camera streaming started', 'success');
        
        document.getElementById('imageElement').style.display = 'block';
        document.getElementById('recordBtn').disabled = false;
        this.updateStatus();
    }

    stopStreaming() {
        this.isStreaming = false;
        this.logActivity('Camera streaming stopped', 'info');
        this.showNotification('Camera streaming stopped', 'info');
        
        document.getElementById('imageElement').style.display = 'none';
        document.getElementById('recordBtn').disabled = true;
        this.updateStatus();
    }

    // Update FPS counter
    updateFPS() {
        if (!this.lastFrameTime) {
            this.lastFrameTime = Date.now();
            this.frameCount = 0;
            return;
        }

        this.frameCount++;
        const currentTime = Date.now();
        const timeDiff = currentTime - this.lastFrameTime;

        if (timeDiff >= 1000) {
            const fps = Math.round(this.frameCount / (timeDiff / 1000));
            document.getElementById('fpsCounter').textContent = fps;
            this.frameCount = 0;
            this.lastFrameTime = currentTime;
        }
    }

    // Enhanced quality change with comprehensive feedback
    changeQuality(quality) {
        this.currentQuality = quality;
        const qualityMap = {
            'low': { width: 640, height: 480, display: '480p' },
            'medium': { width: 1280, height: 720, display: '720p' },
            'high': { width: 1920, height: 1080, display: '1080p' }
        };
        
        const settings = qualityMap[quality];
        
        this.logActivity(`Changing camera quality to ${settings.display}`, 'info');
        this.showNotification(`Quality changed to ${settings.display}`, 'info');
        
        // Send quality change request to ROS2
        this.sendQualityChangeRequest(settings);
        
        document.getElementById('qualityDisplay').textContent = settings.display;
    }

    // Send quality change request with enhanced feedback
    sendQualityChangeRequest(settings) {
        if (!this.ros || !this.ros.isConnected) {
            this.logActivity('Cannot change quality: ROS2 not connected', 'error');
            this.showNotification('Quality change failed: Not connected to ROS2', 'error');
            return;
        }

        // Create service client for camera configuration
        const configService = new ROSLIB.Service({
            ros: this.ros,
            name: '/camera/set_parameters',
            serviceType: 'rcl_interfaces/srv/SetParameters'
        });

        const request = new ROSLIB.ServiceRequest({
            parameters: [
                {
                    name: 'width',
                    value: {
                        type: 2, // INTEGER
                        integer_value: settings.width
                    }
                },
                {
                    name: 'height',
                    value: {
                        type: 2, // INTEGER
                        integer_value: settings.height
                    }
                }
            ]
        });

        configService.callService(request, (result) => {
            if (result.results && result.results.length > 0) {
                const success = result.results.every(r => r.successful);
                if (success) {
                    this.logActivity(`Camera quality successfully updated to ${settings.display}`, 'success');
                    this.showNotification(`Quality updated to ${settings.display}`, 'success');
                } else {
                    this.logActivity(`Camera quality update failed: ${result.results[0].reason}`, 'error');
                    this.showNotification('Quality update failed', 'error');
                }
            } else {
                this.logActivity('Camera configuration service call completed', 'info');
            }
        }, (error) => {
            this.logActivity(`Quality change service call failed: ${error}`, 'error');
            this.showNotification('Quality change failed', 'error');
        });
    }

    // Recording toggle method
    toggleRecording() {
        if (!this.isRecording) {
            this.startRecording();
        } else {
            this.stopRecording();
        }
    }

    // Enhanced recording controls with comprehensive feedback
    startRecording() {
        if (!this.isStreaming) {
            this.logActivity('Cannot start recording: Camera not streaming', 'error');
            this.showNotification('Please start streaming first', 'error');
            return;
        }

        // Send recording start command to ROS2
        this.sendRecordingCommand(true);
        
        this.isRecording = true;
        this.recordingStartTime = Date.now();
        
        this.logActivity('Recording started', 'success');
        this.showNotification('Recording started', 'success');
        
        document.querySelector('.recording-controls').style.display = 'flex';
        document.getElementById('recordBtn').classList.add('recording');
        document.getElementById('recordBtn').innerHTML = '<i class="icon-stop"></i> Recording...';
        
        this.updateRecordingTime();
    }

    stopRecording() {
        // Send recording stop command to ROS2
        this.sendRecordingCommand(false);
        
        this.isRecording = false;
        
        const recordingDuration = Math.round((Date.now() - this.recordingStartTime) / 1000);
        this.logActivity(`Recording stopped (Duration: ${recordingDuration}s)`, 'info');
        this.showNotification(`Recording saved (${recordingDuration}s)`, 'success');
        
        document.querySelector('.recording-controls').style.display = 'none';
        document.getElementById('recordBtn').classList.remove('recording');
        document.getElementById('recordBtn').innerHTML = '<i class="icon-record"></i> Record';
    }

    // Send recording command to ROS2
    sendRecordingCommand(start) {
        if (!this.ros || !this.ros.isConnected) {
            this.logActivity('Cannot send recording command: ROS2 not connected', 'error');
            return;
        }

        const recordingTopic = new ROSLIB.Topic({
            ros: this.ros,
            name: '/camera/recording_command',
            messageType: 'std_msgs/msg/Bool'
        });

        const message = new ROSLIB.Message({
            data: start
        });

        recordingTopic.publish(message);
        this.logActivity(`Recording command sent: ${start ? 'START' : 'STOP'}`, 'info');
    }

    // Update recording time display
    updateRecordingTime() {
        if (!this.isRecording) return;
        
        const elapsed = Date.now() - this.recordingStartTime;
        const minutes = Math.floor(elapsed / 60000);
        const seconds = Math.floor((elapsed % 60000) / 1000);
        
        document.getElementById('recordingTime').textContent = 
            `${minutes.toString().padStart(2, '0')}:${seconds.toString().padStart(2, '0')}`;
        
        setTimeout(() => this.updateRecordingTime(), 1000);
    }

    // Fullscreen toggle functionality
    toggleFullscreen() {
        const viewport = document.getElementById('cameraViewport');
        
        if (!this.isFullscreen) {
            if (viewport.requestFullscreen) {
                viewport.requestFullscreen();
            } else if (viewport.webkitRequestFullscreen) {
                viewport.webkitRequestFullscreen();
            } else if (viewport.mozRequestFullScreen) {
                viewport.mozRequestFullScreen();
            }
            this.isFullscreen = true;
            this.logActivity('Entered fullscreen mode', 'info');
        } else {
            if (document.exitFullscreen) {
                document.exitFullscreen();
            } else if (document.webkitExitFullscreen) {
                document.webkitExitFullscreen();
            } else if (document.mozCancelFullScreen) {
                document.mozCancelFullScreen();
            }
            this.isFullscreen = false;
            this.logActivity('Exited fullscreen mode', 'info');
        }
    }

    // Comprehensive event binding including all controls
    bindEvents() {
        // Quality selector
        document.getElementById('qualitySelect').addEventListener('change', (e) => {
            this.changeQuality(e.target.value);
        });

        // Connect button
        document.getElementById('connectBtn').addEventListener('click', () => {
            if (this.isStreaming) {
                this.stopStreaming();
                document.getElementById('connectBtn').innerHTML = '<i class="icon-connect"></i> Connect';
            } else {
                this.startStreaming();
                document.getElementById('connectBtn').innerHTML = '<i class="icon-disconnect"></i> Disconnect';
            }
        });

        // Record button
        document.getElementById('recordBtn').addEventListener('click', () => {
            this.toggleRecording();
        });

        // Fullscreen button
        document.getElementById('fullscreenBtn').addEventListener('click', () => {
            this.toggleFullscreen();
        });

        // Stop recording button
        document.getElementById('stopRecordBtn').addEventListener('click', () => {
            this.stopRecording();
        });

        // Log panel controls
        document.getElementById('showLogBtn').addEventListener('click', () => {
            document.getElementById('activityLog').style.display = 'block';
            document.querySelector('.log-toggle-container').style.display = 'none';
            this.logActivity('Activity log opened', 'info');
        });

        document.getElementById('toggleLogBtn').addEventListener('click', () => {
            document.getElementById('activityLog').style.display = 'none';
            document.querySelector('.log-toggle-container').style.display = 'block';
            this.logActivity('Activity log closed', 'info');
        });

        document.getElementById('clearLogBtn').addEventListener('click', () => {
            document.getElementById('logContent').innerHTML = '';
            this.logActivity('Activity log cleared', 'info');
        });

        // Fullscreen change detection
        document.addEventListener('fullscreenchange', () => {
            this.isFullscreen = !!document.fullscreenElement;
        });
    }

    // Update status indicators
    updateStatus() {
        const statusIndicator = document.querySelector('.status-indicator');
        const statusText = document.querySelector('.status-text');
        
        if (this.isStreaming) {
            statusIndicator.classList.remove('offline');
            statusIndicator.classList.add('streaming');
            statusText.textContent = 'LIVE';
        } else {
            statusIndicator.classList.remove('streaming');
            statusIndicator.classList.add('offline');
            statusText.textContent = 'OFFLINE';
        }
    }

    // Clean up resources
    destroy() {
        this.logActivity('Destroying ROS2CameraFeed instance', 'info');
        
        this.isStreaming = false;
        this.isRecording = false;
        
        if (this.imageSubscriber) {
            this.imageSubscriber.unsubscribe();
        }
        if (this.cameraInfoSubscriber) {
            this.cameraInfoSubscriber.unsubscribe();
        }
        if (this.ros) {
            this.ros.close();
        }
        
        this.showNotification('Camera feed disconnected', 'info');
    }
}

// Export for use in other modules
if (typeof module !== 'undefined' && module.exports) {
    module.exports = ROS2CameraFeed;
}

// Usage example:
// const cameraFeed = new ROS2CameraFeed('camera-container');