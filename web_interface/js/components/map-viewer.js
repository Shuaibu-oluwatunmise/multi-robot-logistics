class MapViewer {
    constructor() {
        this.element = null;
        this.robotPositions = new Map();
        this.pickupLocation = null;
        this.destinationLocation = null;
        this.clickState = 'pickup'; // 'pickup' or 'destination'
        this.mapBounds = { min: -5, max: 5 }; // Map coordinate bounds
    }

    render() {
        const mapHTML = `
            <div class="map-section">
                <h2 style="margin-bottom: 1rem;">🗺️ Live Map View</h2>
                <div class="map-canvas" id="map-canvas">
                    <div class="map-placeholder" id="map-placeholder">
                        Click to set pickup location, then destination<br>
                        <small>Robots will appear here once connected</small>
                    </div>
                </div>
                
                <!-- Robot Controls moved inside map section -->
                <div style="display: flex; gap: 2rem; justify-content: center; margin-top: 1rem;" id="controls-container">
                    <!-- Control pads will be dynamically added here -->
                </div>
            </div>
        `;

        const tempDiv = document.createElement('div');
        tempDiv.innerHTML = mapHTML;
        this.element = tempDiv.firstElementChild;
        
        this.bindEvents();
        
        return this.element;
    }

    bindEvents() {
        if (!this.element) return;

        const mapCanvas = this.element.querySelector('#map-canvas');
        mapCanvas.addEventListener('click', (event) => this.handleMapClick(event));
    }

    handleMapClick(event) {
        const mapCanvas = event.currentTarget;
        const rect = mapCanvas.getBoundingClientRect();
        
        // Convert pixel coordinates to map coordinates
        const x = ((event.clientX - rect.left) / rect.width) * 
                  (this.mapBounds.max - this.mapBounds.min) + this.mapBounds.min;
        const y = ((event.clientY - rect.top) / rect.height) * 
                  (this.mapBounds.max - this.mapBounds.min) + this.mapBounds.min;

        const roundedX = x.toFixed(2);
        const roundedY = y.toFixed(2);

        if (this.clickState === 'pickup') {
            this.setPickupLocation(parseFloat(roundedX), parseFloat(roundedY));
            this.clickState = 'destination';
            this.log(`Pickup location set: (${roundedX}, ${roundedY})`, 'info');
        } else {
            this.setDestinationLocation(parseFloat(roundedX), parseFloat(roundedY));
            this.clickState = 'pickup'; // Reset for next ride
            this.log(`Destination set: (${roundedX}, ${roundedY})`, 'info');
            
            // Update ride controls
            this.updateRideControls();
        }

        this.updateMapVisualization();
    }

    setPickupLocation(x, y) {
        this.pickupLocation = { x, y };
        
        // Update pickup display
        const pickupDisplay = document.getElementById('pickup-display');
        if (pickupDisplay) {
            pickupDisplay.textContent = `(${x}, ${y})`;
        }
    }

    setDestinationLocation(x, y) {
        this.destinationLocation = { x, y };
        
        // Update destination display
        const destinationDisplay = document.getElementById('destination-display');
        if (destinationDisplay) {
            destinationDisplay.textContent = `(${x}, ${y})`;
        }
    }

    updateRobotPosition(robotId, position) {
        this.robotPositions.set(robotId, position);
        this.updateMapVisualization();
    }

    updateMapVisualization() {
        const mapCanvas = this.element.querySelector('#map-canvas');
        if (!mapCanvas) return;

        // Clear existing markers
        const existingMarkers = mapCanvas.querySelectorAll('.robot-position, .pickup-pin, .destination-pin');
        existingMarkers.forEach(marker => marker.remove());

        // Hide placeholder if we have robots or locations
        const placeholder = mapCanvas.querySelector('#map-placeholder');
        const hasContent = this.robotPositions.size > 0 || this.pickupLocation || this.destinationLocation;
        if (placeholder) {
            placeholder.style.display = hasContent ? 'none' : 'flex';
        }

        // Add robot positions
        this.robotPositions.forEach((position, robotId) => {
            this.addRobotMarker(robotId, position.x, position.y);
        });

        // Add pickup/destination pins
        if (this.pickupLocation) {
            this.addPin(this.pickupLocation.x, this.pickupLocation.y, 'pickup-pin');
        }
        if (this.destinationLocation) {
            this.addPin(this.destinationLocation.x, this.destinationLocation.y, 'destination-pin');
        }
    }

    addRobotMarker(robotId, x, y) {
        const mapCanvas = this.element.querySelector('#map-canvas');
        if (!mapCanvas) return;

        const marker = document.createElement('div');
        marker.className = `robot-position ${robotId.replace('_', '-')}`;
        
        // Convert world coordinates to canvas coordinates (percentage)
        const canvasX = ((x - this.mapBounds.min) / (this.mapBounds.max - this.mapBounds.min)) * 100;
        const canvasY = ((y - this.mapBounds.min) / (this.mapBounds.max - this.mapBounds.min)) * 100;
        
        marker.style.left = `${canvasX}%`;
        marker.style.top = `${canvasY}%`;
        marker.title = `${robotId}: (${x.toFixed(2)}, ${y.toFixed(2)})`;
        
        mapCanvas.appendChild(marker);
    }

    addPin(x, y, className) {
        const mapCanvas = this.element.querySelector('#map-canvas');
        if (!mapCanvas) return;

        const pin = document.createElement('div');
        pin.className = className;
        
        // Convert world coordinates to canvas coordinates
        const canvasX = ((x - this.mapBounds.min) / (this.mapBounds.max - this.mapBounds.min)) * 100;
        const canvasY = ((y - this.mapBounds.min) / (this.mapBounds.max - this.mapBounds.min)) * 100;
        
        pin.style.left = `${canvasX}%`;
        pin.style.top = `${canvasY}%`;
        pin.title = `${className.replace('-pin', '')}: (${x}, ${y})`;
        
        mapCanvas.appendChild(pin);
    }

    addControlPad(controlPad) {
        const controlsContainer = this.element.querySelector('#controls-container');
        if (controlsContainer && controlPad.element) {
            controlsContainer.appendChild(controlPad.element);
        }
    }

    removeControlPad(robotId) {
        const controlsContainer = this.element.querySelector('#controls-container');
        const controlElement = controlsContainer?.querySelector(`#${robotId}-control`);
        if (controlElement) {
            controlElement.remove();
        }
    }

    updateRideControls() {
        // Update request ride button state
        const requestBtn = document.getElementById('request-ride-btn');
        if (requestBtn) {
            if (this.pickupLocation && this.destinationLocation) {
                requestBtn.disabled = false;
                requestBtn.textContent = 'Request Ride';
            } else {
                requestBtn.disabled = true;
                requestBtn.textContent = this.pickupLocation ? 'Set Destination' : 'Set Pickup Location';
            }
        }
    }

    clearLocations() {
        this.pickupLocation = null;
        this.destinationLocation = null;
        this.clickState = 'pickup';
        
        // Clear displays
        const pickupDisplay = document.getElementById('pickup-display');
        const destinationDisplay = document.getElementById('destination-display');
        
        if (pickupDisplay) pickupDisplay.textContent = 'Click on map to set pickup';
        if (destinationDisplay) destinationDisplay.textContent = 'Click on map to set destination';
        
        this.updateMapVisualization();
        this.updateRideControls();
    }

    getPickupLocation() {
        return this.pickupLocation;
    }

    getDestinationLocation() {
        return this.destinationLocation;
    }

    setMapBounds(minBound, maxBound) {
        this.mapBounds = { min: minBound, max: maxBound };
        this.updateMapVisualization();
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
window.MapViewer = MapViewer;
