/**
 * ROS2WebInterface
 * Main class for managing the web interface, topics, and event listeners.
 * Integrates with ROS2ServiceManager for service calls.
 */

// If using modules, import the service manager:
// import ROS2ServiceManager from './service-manager.js';

class ROS2WebInterface {
    constructor() {
        this.ros = null;
        this.connected = false;
        this.cmdVelTopic = null;
        this.odomTopic = null;
        this.currentSpeed = 0.5;
        this.serviceManager = null; // Service manager instance

        this.init();
    }

    async init() {
        this.setupEventListeners();
        this.connectToROS2();
    }

    connectToROS2() {
        // Try Foxglove Bridge first, then fallback to rosbridge
        const bridgeURLs = [
            'ws://localhost:8765',  // Foxglove Bridge
            'ws://localhost:9090'   // rosbridge_suite
        ];

        this.tryConnection(bridgeURLs, 0);
    }

    tryConnection(urls, index) {
        if (index >= urls.length) {
            this.updateConnectionStatus('error', 'Failed to connect to any bridge');
            return;
        }

        console.log(`Attempting connection to ${urls[index]}`);

        this.ros = new ROSLIB.Ros({
            url: urls[index]
        });

        this.ros.on('connection', () => {
            console.log('Connected to ROS2 bridge');
            this.connected = true;
            this.updateConnectionStatus('connected', 'Connected to ROS2');
            this.setupTopics();
            this.setupServices(); // Initialize service manager and services
        });

        this.ros.on('error', (error) => {
            console.log('Connection error:', error);
            this.connected = false;
            this.updateConnectionStatus('error', `Connection error: ${error}`);

            // Try next URL after a short delay
            setTimeout(() => {
                this.tryConnection(urls, index + 1);
            }, 2000);
        });

        this.ros.on('close', () => {
            console.log('Connection closed');
            this.connected = false;
            this.updateConnectionStatus('disconnected', 'Disconnected');

            // Attempt reconnection after 5 seconds
            setTimeout(() => {
                this.connectToROS2();
            }, 5000);
        });
    }

    /**
     * Initialize service manager and register common services.
     */
    setupServices() {
        // Initialize service manager when connection is established
        this.serviceManager = new ROS2ServiceManager(this.ros);

        // Setup common services
        this.serviceManager.createServiceClient('/emergency_stop', 'std_srvs/srv/Empty');
        this.serviceManager.createServiceClient('/reset_position', 'geometry_msgs/srv/SetPosition');
        this.serviceManager.createServiceClient('/get_robot_status', 'robot_msgs/srv/GetStatus');
        this.serviceManager.createServiceClient('/set_robot_mode', 'robot_msgs/srv/SetMode');

        console.log('ROS2 services initialized');
    }

    /**
     * Set up ROS2 topics for publishing/subscribing.
     */
    setupTopics() {
        // Set up cmd_vel topic for robot control
        this.cmdVelTopic = new ROSLIB.Topic({
            ros: this.ros,
            name: '/cmd_vel',
            messageType: 'geometry_msgs/msg/Twist'
        });

        // Set up odometry topic for position feedback
        this.odomTopic = new ROSLIB.Topic({
            ros: this.ros,
            name: '/odom',
            messageType: 'nav_msgs/msg/Odometry'
        });

        this.odomTopic.subscribe((message) => {
            this.updatePositionDisplay(message);
        });



        console.log('ROS2 topics initialized');
    }

    /**
     * Set up UI event listeners for buttons, forms, and keyboard.
     */
    setupEventListeners() {
        // Movement control buttons
        document.getElementById('btn-forward').addEventListener('click', () => this.moveRobot('forward'));
        document.getElementById('btn-backward').addEventListener('click', () => this.moveRobot('backward'));
        document.getElementById('btn-left').addEventListener('click', () => this.moveRobot('left'));
        document.getElementById('btn-right').addEventListener('click', () => this.moveRobot('right'));
        document.getElementById('btn-stop').addEventListener('click', () => this.stopRobot());

        // Emergency stop button (using service)
        document.getElementById('btn-emergency').addEventListener('click', () => this.emergencyStop());

        // Reset position button (using service)
        document.getElementById('btn-reset').addEventListener('click', () => this.resetPosition());

        // Speed control
        const speedSlider = document.getElementById('speed-slider');
        speedSlider.addEventListener('input', (e) => {
            this.currentSpeed = parseFloat(e.target.value);
            document.getElementById('speed-value').textContent = this.currentSpeed.toFixed(1);
        });

        // Control mode toggle
        const controlModeToggle = document.getElementById('controlModeToggle');
        if (controlModeToggle) {
            controlModeToggle.addEventListener('change', (e) => {
                const mode = e.target.checked ? 'Autonomous' : 'Manual';
                this.setRobotMode(mode);
            });
        }

        // Manual control form
        document.getElementById('manual-form').addEventListener('submit', (e) => {
            e.preventDefault();
            this.sendManualCommand();
        });

        // Keyboard controls
        document.addEventListener('keydown', (e) => {
            if (e.target.tagName === 'INPUT') return;

            switch(e.key) {
                // WASD keys for robot movement
                case 'w':
                case 'W':
                    e.preventDefault();
                    this.moveRobot('forward');
                    break;
                case 's':
                case 'S':
                    e.preventDefault();
                    this.moveRobot('backward');
                    break;
                case 'a':
                case 'A':
                    e.preventDefault();
                    this.moveRobot('left');
                    break;
                case 'd':
                case 'D':
                    e.preventDefault();
                    this.moveRobot('right');
                    break;
                // Arrow keys for camera pan-tilt movement
                case 'ArrowUp':
                    e.preventDefault();
                    this.moveCamera('up');
                    break;
                case 'ArrowDown':
                    e.preventDefault();
                    this.moveCamera('down');
                    break;
                case 'ArrowLeft':
                    e.preventDefault();
                    this.moveCamera('left');
                    break;
                case 'ArrowRight':
                    e.preventDefault();
                    this.moveCamera('right');
                    break;
                case ' ':
                case 'Escape':
                    e.preventDefault();
                    this.emergencyStop(); // Use service for emergency stop
                    break;
            }
        });

        // Stop robot when WASD keys are released
        document.addEventListener('keyup', (e) => {
            if (['w', 'W', 's', 'S', 'a', 'A', 'd', 'D'].includes(e.key)) {
                setTimeout(() => this.stopRobot(), 100);
            }
        });
    }

    /**
     * Publish a Twist message to move the robot.
     * @param {string} direction - forward, backward, left, right
     */
    moveRobot(direction) {
        if (!this.connected || !this.cmdVelTopic) {
            console.warn('Not connected to ROS2');
            return;
        }

        let linear = { x: 0.0, y: 0.0, z: 0.0 };
        let angular = { x: 0.0, y: 0.0, z: 0.0 };

        switch(direction) {
            case 'forward':
                linear.x = this.currentSpeed;
                break;
            case 'backward':
                linear.x = -this.currentSpeed;
                break;
            case 'left':
                angular.z = this.currentSpeed;
                break;
            case 'right':
                angular.z = -this.currentSpeed;
                break;
        }

        const twist = new ROSLIB.Message({
            linear: linear,
            angular: angular
        });

        this.cmdVelTopic.publish(twist);
        console.log(`Moving ${direction} at speed ${this.currentSpeed}`);
    }

    /**
     * Publish a zero Twist message to stop the robot.
     */
    stopRobot() {
        if (!this.connected || !this.cmdVelTopic) return;

        const stopTwist = new ROSLIB.Message({
            linear: { x: 0.0, y: 0.0, z: 0.0 },
            angular: { x: 0.0, y: 0.0, z: 0.0 }
        });

        this.cmdVelTopic.publish(stopTwist);
        console.log('Robot stopped');
    }

    /**
     * Handle camera pan-tilt movement via keyboard.
     * @param {string} direction - up, down, left, right
     */
    moveCamera(direction) {
        if (!this.connected) {
            console.warn('Not connected to ROS2');
            return;
        }

        console.log(`Camera moving ${direction}`);
        
        // Here you would typically publish to a camera control topic
        // For now, we'll just log the movement
        // Example: this.cameraTopic.publish(new ROSLIB.Message({ direction: direction }));
        
        // Update UI to show camera movement
        this.updateConnectionStatus('connected', `Camera: ${direction}`);
    }

    // Service-based methods

    /**
     * Emergency stop using service, fallback to topic if unavailable.
     */
    async emergencyStop() {
        if (!this.serviceManager) {
            console.warn('Service manager not initialized');
            this.stopRobot(); // Fallback to topic-based stop
            return;
        }

        try {
            await this.serviceManager.emergencyStop();
            this.updateConnectionStatus('connected', 'Emergency stop activated');
        } catch (error) {
            console.error('Emergency stop service failed, using topic fallback');
            this.stopRobot();
        }
    }

    /**
     * Reset robot position using service.
     */
    async resetPosition() {
        if (!this.serviceManager) {
            console.warn('Service manager not initialized');
            return;
        }

        try {
            await this.serviceManager.resetPosition();
            this.updateConnectionStatus('connected', 'Position reset');
        } catch (error) {
            console.error('Reset position failed:', error);
        }
    }

    /**
     * Get robot status using service.
     */
    async getRobotStatus() {
        if (!this.serviceManager) return null;

        try {
            const status = await this.serviceManager.getRobotStatus();
            console.log('Robot status:', status);
            return status;
        } catch (error) {
            console.error('Failed to get robot status:', error);
            return null;
        }
    }

    /**
     * Set robot mode using service.
     */
    async setRobotMode(mode) {
        if (!this.serviceManager) {
            console.error('Service manager not initialized');
            return;
        }

        try {
            const response = await this.serviceManager.callService('/set_robot_mode', {
                mode: mode.toLowerCase()
            });
            console.log(`Robot mode set to ${mode}:`, response);
            this.updateConnectionStatus('connected', `Mode: ${mode}`);
        } catch (error) {
            console.error(`Failed to set robot mode to ${mode}:`, error);
            this.updateConnectionStatus('error', `Failed to set mode: ${mode}`);
        }
    }

    /**
     * Send manual velocity command from form input.
     */
    sendManualCommand() {
        if (!this.connected || !this.cmdVelTopic) return;

        const linearX = parseFloat(document.getElementById('manual-linear').value);
        const angularZ = parseFloat(document.getElementById('manual-angular').value);

        const manualTwist = new ROSLIB.Message({
            linear: { x: linearX, y: 0.0, z: 0.0 },
            angular: { x: 0.0, y: 0.0, z: angularZ }
        });

        this.cmdVelTopic.publish(manualTwist);
        console.log(`Manual command: linear=${linearX}, angular=${angularZ}`);
    }

    /**
     * Update connection status indicator in the UI.
     * @param {string} status
     * @param {string} message
     */
    updateConnectionStatus(status, message) {
        const indicator = document.getElementById('status-indicator');
        const text = document.getElementById('status-text');

        indicator.className = `status-${status}`;
        text.textContent = message;
    }

    /**
     * Update robot position and velocity display from odometry.
     * @param {object} odomMessage
     */
    updatePositionDisplay(odomMessage) {
        const position = odomMessage.pose.pose.position;
        const orientation = odomMessage.pose.pose.orientation;
        const velocity = odomMessage.twist.twist;

        // Convert quaternion to euler angle (simplified for Z rotation)
        const theta = Math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
        );

        // Update position display
        document.getElementById('pos-x').textContent = position.x.toFixed(2);
        document.getElementById('pos-y').textContent = position.y.toFixed(2);
        document.getElementById('pos-theta').textContent = theta.toFixed(2);

        // Update velocity display
        document.getElementById('vel-linear').textContent = velocity.linear.x.toFixed(2);
        document.getElementById('vel-angular').textContent = velocity.angular.z.toFixed(2);
    }


}

// Initialize the interface when the page loads
document.addEventListener('DOMContentLoaded', () => {
    const robotInterface = new ROS2WebInterface();

    // Add global reference for debugging
    window.robotInterface = robotInterface;

    console.log('ROS2 Web Interface with Service Manager initialized');
});