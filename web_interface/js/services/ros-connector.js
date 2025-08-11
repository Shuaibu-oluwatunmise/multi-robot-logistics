class ROSConnector {
    constructor() {
        this.connections = new Map();
        this.subscribers = new Map();
        this.publishers = new Map();
        this.moveIntervals = new Map();
    }

    async connectRobot(robotConfig) {
        const { id, rosbridge_port, name } = robotConfig;
        
        return new Promise((resolve, reject) => {
            const ros = new ROSLIB.Ros({
                url: `ws://localhost:${rosbridge_port}`
            });

            ros.on('connection', () => {
                this.connections.set(id, ros);
                this.setupSubscriptions(robotConfig, ros);
                this.setupPublishers(robotConfig, ros);
                this.log(`${name} connected to ROSBridge (port ${rosbridge_port})`, 'info');
                resolve(ros);
            });

            ros.on('error', (error) => {
                this.log(`${name} ROSBridge connection error: ${error}`, 'error');
                reject(error);
            });

            ros.on('close', () => {
                this.log(`${name} ROSBridge connection closed`, 'warning');
                this.connections.delete(id);
                // Auto-reconnect after 3 seconds
                setTimeout(() => {
                    this.connectRobot(robotConfig);
                }, 3000);
            });
        });
    }

    setupSubscriptions(robotConfig, ros) {
        const { id } = robotConfig;
        
        // Odometry subscription
        const odomTopic = new ROSLIB.Topic({
            ros: ros,
            name: '/odom',
            messageType: 'nav_msgs/Odometry'
        });

        odomTopic.subscribe((message) => {
            this.handleOdometryUpdate(id, message);
        });

        this.subscribers.set(`${id}_odom`, odomTopic);
    }

    setupPublishers(robotConfig, ros) {
        const { id } = robotConfig;
        
        // Command velocity publisher
        const cmdVelTopic = new ROSLIB.Topic({
            ros: ros,
            name: '/cmd_vel',
            messageType: 'geometry_msgs/TwistStamped'
        });

        this.publishers.set(`${id}_cmd_vel`, cmdVelTopic);
    }

    handleOdometryUpdate(robotId, message) {
        const x = message.pose.pose.position.x.toFixed(3);
        const y = message.pose.pose.position.y.toFixed(3);
        
        // Update robot position in UI
        window.robotManager?.updateRobotPosition(robotId, { x: parseFloat(x), y: parseFloat(y) });
        
        // Update position display
        this.updatePositionDisplay(robotId, x, y);
    }

    updatePositionDisplay(robotId, x, y) {
        const xElement = document.getElementById(`${robotId}-x`);
        const yElement = document.getElementById(`${robotId}-y`);
        
        if (xElement) xElement.textContent = x;
        if (yElement) yElement.textContent = y;
    }

    startMovement(robotId, linear, angular) {
        // Clear any existing movement interval
        this.stopMovement(robotId);
        
        // Start continuous movement
        const interval = setInterval(() => {
            this.sendMoveCommand(robotId, linear, angular);
        }, 100);
        
        this.moveIntervals.set(robotId, interval);
        this.log(`${robotId} moving: linear=${linear}, angular=${angular}`, 'info');
    }

    stopMovement(robotId) {
        // Clear movement interval
        const interval = this.moveIntervals.get(robotId);
        if (interval) {
            clearInterval(interval);
            this.moveIntervals.delete(robotId);
        }
        
        // Send stop command
        this.sendMoveCommand(robotId, 0, 0);
        this.log(`${robotId} stopped`, 'info');
    }

    sendMoveCommand(robotId, linear, angular) {
        const publisher = this.publishers.get(`${robotId}_cmd_vel`);
        if (!publisher) return;

        const twistStamped = new ROSLIB.Message({
            header: {
                stamp: {
                    sec: Math.floor(Date.now() / 1000),
                    nanosec: (Date.now() % 1000) * 1000000
                },
                frame_id: ''
            },
            twist: {
                linear: {
                    x: linear,
                    y: 0,
                    z: 0
                },
                angular: {
                    x: 0,
                    y: 0,
                    z: angular
                }
            }
        });

        publisher.publish(twistStamped);
    }

    disconnectRobot(robotId) {
        const ros = this.connections.get(robotId);
        if (ros) {
            ros.close();
            this.connections.delete(robotId);
        }
        
        // Clean up subscribers and publishers
        this.subscribers.delete(`${robotId}_odom`);
        this.publishers.delete(`${robotId}_cmd_vel`);
        this.stopMovement(robotId);
    }

    isConnected(robotId) {
        const ros = this.connections.get(robotId);
        return ros && ros.isConnected;
    }

    log(message, level = 'info') {
        if (window.logger) {
            window.logger.log(message, level);
        } else {
            console.log(`[${level.toUpperCase()}] ${message}`);
        }
    }
}

// Export for global use
window.ROSConnector = ROSConnector;
