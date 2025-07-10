/**
 * ROS2ServiceManager
 * Handles creation and management of ROS2 service clients and service calls.
 * Intended to be used by the main ROS2WebInterface.
 */
class ROS2ServiceManager {
    /**
     * @param {ROSLIB.Ros} ros - The ROS connection instance.
     */
    constructor(ros) {
        this.ros = ros;
        this.services = new Map();
    }

    /**
     * Create and store a service client.
     * @param {string} serviceName - Name of the ROS2 service.
     * @param {string} serviceType - Type of the ROS2 service.
     * @returns {ROSLIB.Service}
     */
    createServiceClient(serviceName, serviceType) {
        const service = new ROSLIB.Service({
            ros: this.ros,
            name: serviceName,
            serviceType: serviceType
        });

        this.services.set(serviceName, service);
        return service;
    }

    /**
     * Call a service by name with a request object.
     * @param {string} serviceName
     * @param {object} request
     * @returns {Promise<object>}
     */
    async callService(serviceName, request) {
        const service = this.services.get(serviceName);
        if (!service) {
            throw new Error(`Service ${serviceName} not found`);
        }

        return new Promise((resolve, reject) => {
            const serviceRequest = new ROSLIB.ServiceRequest(request);
            service.callService(serviceRequest, (result) => {
                resolve(result);
            }, (error) => {
                reject(error);
            });
        });
    }

    // Example: Emergency stop service
    async emergencyStop() {
        try {
            const result = await this.callService('/emergency_stop', {});
            console.log('Emergency stop activated:', result);
            return result;
        } catch (error) {
            console.error('Emergency stop failed:', error);
            throw error;
        }
    }

    // Example: Reset robot position
    async resetPosition() {
        try {
            const result = await this.callService('/reset_position', {
                x: 0.0,
                y: 0.0,
                theta: 0.0
            });
            console.log('Position reset:', result);
            return result;
        } catch (error) {
            console.error('Position reset failed:', error);
            throw error;
        }
    }

    // Get robot status
    async getRobotStatus() {
        try {
            const result = await this.callService('/get_robot_status', {});
            return result;
        } catch (error) {
            console.error('Failed to get robot status:', error);
            throw error;
        }
    }

    // Set robot mode (manual, autonomous, etc.)
    async setRobotMode(mode) {
        try {
            const result = await this.callService('/set_robot_mode', { mode: mode });
            console.log(`Robot mode set to: ${mode}`);
            return result;
        } catch (error) {
            console.error('Failed to set robot mode:', error);
            throw error;
        }
    }
}

// Export for use in interface.js if using modules
export default ROS2ServiceManager;