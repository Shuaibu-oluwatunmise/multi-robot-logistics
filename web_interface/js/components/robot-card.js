class RobotCard {
    constructor(robotConfig) {
        this.robot = robotConfig;
        this.element = null;
        this.isConnected = false;
    }

    render() {
        const cardHTML = `
            <div class="robot-card" id="${this.robot.id}-card">
                <div class="robot-header">
                    <div class="robot-name">🤖 ${this.robot.name}</div>
                    <div class="robot-state available" id="${this.robot.id}-state">AVAILABLE</div>
                </div>
                <div class="robot-stats">
                    <div class="stat">
                        <div class="stat-label">Position X</div>
                        <div class="stat-value" id="${this.robot.id}-x">--</div>
                    </div>
                    <div class="stat">
                        <div class="stat-label">Position Y</div>
                        <div class="stat-value" id="${this.robot.id}-y">--</div>
                    </div>
                    <div class="stat">
                        <div class="stat-label">Battery</div>
                        <div class="stat-value" id="${this.robot.id}-battery">--</div>
                    </div>
                    <div class="stat">
                        <div class="stat-label">Status</div>
                        <div class="stat-value" id="${this.robot.id}-status">Idle</div>
                    </div>
                </div>
                <div class="robot-actions">
                    <button class="btn btn-small" onclick="window.robotManager.toggleConnection('${this.robot.id}')" id="${this.robot.id}-connect-btn">
                        Connect
                    </button>
                    <button class="btn btn-small btn-danger" onclick="window.robotManager.removeRobot('${this.robot.id}')" style="background: rgba(255, 107, 107, 0.6); margin-top: 0.5rem;">
                        Remove
                    </button>
                </div>
            </div>
        `;

        const tempDiv = document.createElement('div');
        tempDiv.innerHTML = cardHTML;
        this.element = tempDiv.firstElementChild;
        
        return this.element;
    }

    updateConnectionStatus(connected) {
        this.isConnected = connected;
        const stateElement = this.element.querySelector(`#${this.robot.id}-state`);
        const connectBtn = this.element.querySelector(`#${this.robot.id}-connect-btn`);
        const statusElement = this.element.querySelector(`#${this.robot.id}-status`);

        if (connected) {
            stateElement.textContent = 'CONNECTED';
            stateElement.className = 'robot-state available';
            connectBtn.textContent = 'Disconnect';
            statusElement.textContent = 'Connected';
        } else {
            stateElement.textContent = 'DISCONNECTED';
            stateElement.className = 'robot-state busy';
            connectBtn.textContent = 'Connect';
            statusElement.textContent = 'Disconnected';
            
            // Reset position displays
            this.element.querySelector(`#${this.robot.id}-x`).textContent = '--';
            this.element.querySelector(`#${this.robot.id}-y`).textContent = '--';
        }
    }

    updatePosition(x, y) {
        if (this.element) {
            this.element.querySelector(`#${this.robot.id}-x`).textContent = x.toFixed(3);
            this.element.querySelector(`#${this.robot.id}-y`).textContent = y.toFixed(3);
        }
    }

    updateBattery(batteryLevel) {
        if (this.element) {
            const batteryElement = this.element.querySelector(`#${this.robot.id}-battery`);
            if (batteryLevel !== undefined && batteryLevel !== null) {
                batteryElement.textContent = `${batteryLevel}%`;
            } else {
                batteryElement.textContent = '--';
            }
        }
    }

    updateStatus(status) {
        if (this.element) {
            const statusElement = this.element.querySelector(`#${this.robot.id}-status`);
            statusElement.textContent = status;
        }
    }

    setState(state) {
        if (this.element) {
            const stateElement = this.element.querySelector(`#${this.robot.id}-state`);
            stateElement.textContent = state.toUpperCase();
            
            // Update state styling
            stateElement.className = `robot-state ${state === 'available' ? 'available' : 'busy'}`;
        }
    }

    destroy() {
        if (this.element && this.element.parentNode) {
            this.element.parentNode.removeChild(this.element);
        }
    }

    getConfig() {
        return this.robot;
    }

    updateConfig(newConfig) {
        this.robot = { ...this.robot, ...newConfig };
        
        // Update name display if changed
        if (this.element) {
            const nameElement = this.element.querySelector('.robot-name');
            nameElement.textContent = `🤖 ${this.robot.name}`;
        }
    }
}

// Export for global use
window.RobotCard = RobotCard;
