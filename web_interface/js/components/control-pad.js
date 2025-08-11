class ControlPad {
    constructor(robotId, robotName, robotColor, rosConnector) {
        this.robotId = robotId;
        this.robotName = robotName;
        this.robotColor = robotColor;
        this.rosConnector = rosConnector;
        this.element = null;
        this.enabled = false;
        
        // Movement parameters
        this.linearSpeed = 0.2;
        this.angularSpeed = 0.5;
    }

    render() {
        const controlHTML = `
            <div class="robot-control" id="${this.robotId}-control">
                <h3 style="text-align: center; margin-bottom: 0.8rem; color: ${this.robotColor}; font-size: 1rem;">
                    🤖 ${this.robotName} Controls
                </h3>
                <div class="control-pad">
                    <div class="button-row">
                        <button class="arrow-button" data-action="forward-left">↖️</button>
                        <button class="arrow-button" data-action="forward">⬆️</button>
                        <button class="arrow-button" data-action="forward-right">↗️</button>
                    </div>
                    <div class="button-row">
                        <button class="arrow-button" data-action="left">⬅️</button>
                        <button class="arrow-button stop-button" data-action="stop">⛔</button>
                        <button class="arrow-button" data-action="right">➡️</button>
                    </div>
                    <div class="button-row">
                        <button class="arrow-button" data-action="backward-left">↙️</button>
                        <button class="arrow-button" data-action="backward">⬇️</button>
                        <button class="arrow-button" data-action="backward-right">↘️</button>
                    </div>
                </div>
                <div class="speed-controls" style="margin-top: 1rem;">
                    <div style="display: flex; gap: 0.5rem; align-items: center; margin-bottom: 0.5rem;">
                        <label style="font-size: 0.8rem; min-width: 60px;">Linear:</label>
                        <input type="range" min="0.1" max="0.5" step="0.1" value="${this.linearSpeed}" 
                               style="flex: 1;" id="${this.robotId}-linear-speed">
                        <span style="font-size: 0.8rem; min-width: 40px;" id="${this.robotId}-linear-value">${this.linearSpeed}</span>
                    </div>
                    <div style="display: flex; gap: 0.5rem; align-items: center;">
                        <label style="font-size: 0.8rem; min-width: 60px;">Angular:</label>
                        <input type="range" min="0.1" max="1.0" step="0.1" value="${this.angularSpeed}" 
                               style="flex: 1;" id="${this.robotId}-angular-speed">
                        <span style="font-size: 0.8rem; min-width: 40px;" id="${this.robotId}-angular-value">${this.angularSpeed}</span>
                    </div>
                </div>
            </div>
        `;

        const tempDiv = document.createElement('div');
        tempDiv.innerHTML = controlHTML;
        this.element = tempDiv.firstElementChild;
        
        this.bindEvents();
        this.setEnabled(false); // Start disabled
        
        return this.element;
    }

    bindEvents() {
        if (!this.element) return;

        // Bind movement buttons
        const buttons = this.element.querySelectorAll('.arrow-button');
        buttons.forEach(button => {
            const action = button.getAttribute('data-action');
            
            if (action === 'stop') {
                button.addEventListener('click', () => this.handleStop());
            } else {
                button.addEventListener('mousedown', () => this.handleMoveStart(action));
                button.addEventListener('mouseup', () => this.handleMoveStop());
                button.addEventListener('mouseleave', () => this.handleMoveStop());
                
                // Touch events for mobile
                button.addEventListener('touchstart', (e) => {
                    e.preventDefault();
                    this.handleMoveStart(action);
                });
                button.addEventListener('touchend', (e) => {
                    e.preventDefault();
                    this.handleMoveStop();
                });
            }
        });

        // Bind speed controls
        const linearSlider = this.element.querySelector(`#${this.robotId}-linear-speed`);
        const angularSlider = this.element.querySelector(`#${this.robotId}-angular-speed`);
        const linearValue = this.element.querySelector(`#${this.robotId}-linear-value`);
        const angularValue = this.element.querySelector(`#${this.robotId}-angular-value`);

        linearSlider.addEventListener('input', (e) => {
            this.linearSpeed = parseFloat(e.target.value);
            linearValue.textContent = this.linearSpeed;
        });

        angularSlider.addEventListener('input', (e) => {
            this.angularSpeed = parseFloat(e.target.value);
            angularValue.textContent = this.angularSpeed;
        });
    }

    handleMoveStart(action) {
        if (!this.enabled) return;

        const movements = {
            'forward': { linear: this.linearSpeed, angular: 0 },
            'backward': { linear: -this.linearSpeed, angular: 0 },
            'left': { linear: 0, angular: this.angularSpeed },
            'right': { linear: 0, angular: -this.angularSpeed },
            'forward-left': { linear: this.linearSpeed, angular: this.angularSpeed },
            'forward-right': { linear: this.linearSpeed, angular: -this.angularSpeed },
            'backward-left': { linear: -this.linearSpeed, angular: -this.angularSpeed },
            'backward-right': { linear: -this.linearSpeed, angular: this.angularSpeed }
        };

        const movement = movements[action];
        if (movement && this.rosConnector) {
            this.rosConnector.startMovement(this.robotId, movement.linear, movement.angular);
        }
    }

    handleMoveStop() {
        if (!this.enabled) return;
        
        if (this.rosConnector) {
            this.rosConnector.stopMovement(this.robotId);
        }
    }

    handleStop() {
        if (this.rosConnector) {
            this.rosConnector.stopMovement(this.robotId);
        }
    }

    setEnabled(enabled) {
        this.enabled = enabled;
        
        if (this.element) {
            const buttons = this.element.querySelectorAll('.arrow-button');
            const sliders = this.element.querySelectorAll('input[type="range"]');
            
            buttons.forEach(button => {
                button.disabled = !enabled;
                button.style.opacity = enabled ? '1' : '0.5';
                button.style.cursor = enabled ? 'pointer' : 'not-allowed';
            });
            
            sliders.forEach(slider => {
                slider.disabled = !enabled;
                slider.style.opacity = enabled ? '1' : '0.5';
            });
        }
    }

    updateRobotInfo(name, color) {
        this.robotName = name;
        this.robotColor = color;
        
        if (this.element) {
            const header = this.element.querySelector('h3');
            header.textContent = `🤖 ${this.robotName} Controls`;
            header.style.color = this.robotColor;
        }
    }

    destroy() {
        // Stop any ongoing movement
        this.handleStop();
        
        if (this.element && this.element.parentNode) {
            this.element.parentNode.removeChild(this.element);
        }
    }
}

// Export for global use
window.ControlPad = ControlPad;
