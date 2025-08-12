class MapViewer {
    constructor() {
        this.element = null;
        this.robotPositions = new Map();
        this.mapData = new Map(); // Store actual SLAM map data
        this.currentMapInfo = null; // For coordinate conversion
        this.mapBounds = { min: -5, max: 5 }; // Fallback coordinate bounds
    }

    render() {
        const mapHTML = `
            <div class="map-section">
                <h2 style="margin-bottom: 1rem;">🗺 Live Map View</h2>
                <div class="map-canvas" id="map-canvas">
                    <div class="map-placeholder" id="map-placeholder">
                        Maps will appear here when SLAM is running<br>
                        <small>Start mapping to see robot-built maps</small>
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
        // Remove click handler for pins - manager mode doesn't need ride requests
        mapCanvas.addEventListener('click', (event) => this.handleMapClick(event));
    }

    handleMapClick(event) {
        // Manager mode: Just log coordinates for debugging
        const mapCanvas = event.currentTarget;
        const rect = mapCanvas.getBoundingClientRect();
        
        // Get canvas coordinates
        const canvasX = event.clientX - rect.left;
        const canvasY = event.clientY - rect.top;
        
        // Convert to world coordinates if we have map data
        if (this.currentMapInfo) {
            const worldCoords = this.convertCanvasToMap(canvasX, canvasY);
            this.log(`Map clicked at world coordinates: (${worldCoords.x.toFixed(3)}, ${worldCoords.y.toFixed(3)})`, 'info');
        } else {
            // Fallback to percentage coordinates
            const x = ((canvasX / rect.width) * (this.mapBounds.max - this.mapBounds.min) + this.mapBounds.min).toFixed(2);
            const y = ((canvasY / rect.height) * (this.mapBounds.max - this.mapBounds.min) + this.mapBounds.min).toFixed(2);
            this.log(`Map clicked at estimated coordinates: (${x}, ${y})`, 'info');
        }
    }

    // REAL MAP DATA HANDLING
    updateMapData(robotId, mapData) {
        this.log(`Received map data from ${robotId}`, 'info');
        
        // Store map data
        this.mapData.set(robotId, mapData);
        
        // Render the map
        this.renderMap(mapData);
        
        // Hide placeholder since we now have actual map data
        const placeholder = this.element.querySelector('#map-placeholder');
        if (placeholder) {
            placeholder.style.display = 'none';
        }
    }

    renderMap(mapData) {
        const mapCanvas = this.element.querySelector('#map-canvas');
        if (!mapCanvas) return;
        
        // Create or get the map canvas element
        let canvas = mapCanvas.querySelector('#map-render-canvas');
        if (!canvas) {
            canvas = document.createElement('canvas');
            canvas.id = 'map-render-canvas';
            canvas.style.position = 'absolute';
            canvas.style.top = '0';
            canvas.style.left = '0';
            canvas.style.width = '100%';
            canvas.style.height = '100%';
            mapCanvas.appendChild(canvas);
        }
        
        const ctx = canvas.getContext('2d');
        
        // Set canvas size to match container
        const rect = mapCanvas.getBoundingClientRect();
        canvas.width = rect.width;
        canvas.height = rect.height;
        
        // Map data properties
        const width = mapData.info.width;
        const height = mapData.info.height;
        const resolution = mapData.info.resolution;
        const origin = mapData.info.origin;
        
        // Create image data
        const imageData = ctx.createImageData(width, height);
        const data = imageData.data;
        
        // Convert occupancy grid to image
        for (let i = 0; i < mapData.data.length; i++) {
            const value = mapData.data[i];
            let color;
            
            if (value === -1) {
                // Unknown space - gray
                color = [128, 128, 128, 255];
            } else if (value === 0) {
                // Free space - white
                color = [255, 255, 255, 255];
            } else {
                // Occupied space - black (darker for higher values)
                const intensity = Math.max(0, 255 - (value * 2.55));
                color = [intensity, intensity, intensity, 255];
            }
            
            const pixelIndex = i * 4;
            data[pixelIndex] = color[0]; // R
            data[pixelIndex + 1] = color[1]; // G
            data[pixelIndex + 2] = color[2]; // B
            data[pixelIndex + 3] = color[3]; // A
        }
        
        // Clear canvas and draw map
        ctx.clearRect(0, 0, canvas.width, canvas.height);
        
        // Scale and draw the map to fit the canvas
        const tempCanvas = document.createElement('canvas');
        tempCanvas.width = width;
        tempCanvas.height = height;
        const tempCtx = tempCanvas.getContext('2d');
        tempCtx.putImageData(imageData, 0, 0);
        
        // Draw scaled map
        ctx.drawImage(tempCanvas, 0, 0, canvas.width, canvas.height);
        
        // Store map info for coordinate conversion
        this.currentMapInfo = {
            width: width,
            height: height,
            resolution: resolution,
            origin: origin,
            canvasWidth: canvas.width,
            canvasHeight: canvas.height
        };
        
        // Redraw robot positions on top of map
        setTimeout(() => {
            this.updateMapVisualization();
        }, 100);
    }

    // COORDINATE CONVERSION METHODS
    convertMapToCanvas(x, y) {
        if (!this.currentMapInfo) {
            // Fallback to old coordinate system if no map loaded
            const canvasX = ((x - this.mapBounds.min) / (this.mapBounds.max - this.mapBounds.min)) * 100;
            const canvasY = ((y - this.mapBounds.min) / (this.mapBounds.max - this.mapBounds.min)) * 100;
            return { x: canvasX, y: canvasY };
        }
        
        const info = this.currentMapInfo;
        
        // Convert world coordinates to map grid coordinates
        const mapX = (x - info.origin.position.x) / info.resolution;
        const mapY = (y - info.origin.position.y) / info.resolution;
        
        // Convert map grid coordinates to canvas coordinates
        const canvasX = (mapX / info.width) * info.canvasWidth;
        const canvasY = info.canvasHeight - ((mapY / info.height) * info.canvasHeight); // Flip Y axis
        
        return { x: canvasX, y: canvasY };
    }

    convertCanvasToMap(canvasX, canvasY) {
        if (!this.currentMapInfo) {
            // Fallback to old coordinate system
            const x = (canvasX / 100) * (this.mapBounds.max - this.mapBounds.min) + this.mapBounds.min;
            const y = (canvasY / 100) * (this.mapBounds.max - this.mapBounds.min) + this.mapBounds.min;
            return { x: x, y: y };
        }
        
        const info = this.currentMapInfo;
        
        // Convert canvas coordinates to map grid coordinates
        const mapX = (canvasX / info.canvasWidth) * info.width;
        const mapY = ((info.canvasHeight - canvasY) / info.canvasHeight) * info.height; // Flip Y axis
        
        // Convert map grid coordinates to world coordinates
        const worldX = (mapX * info.resolution) + info.origin.position.x;
        const worldY = (mapY * info.resolution) + info.origin.position.y;
        
        return { x: worldX, y: worldY };
    }

    // ROBOT POSITION TRACKING
    updateRobotPosition(robotId, position) {
        this.robotPositions.set(robotId, position);
        this.updateMapVisualization();
    }

    updateMapVisualization() {
        const mapCanvas = this.element.querySelector('#map-canvas');
        if (!mapCanvas) return;

        // Clear existing robot markers
        const existingMarkers = mapCanvas.querySelectorAll('.robot-position');
        existingMarkers.forEach(marker => marker.remove());

        // Hide placeholder if we have robots or map data
        const placeholder = mapCanvas.querySelector('#map-placeholder');
        const hasContent = this.robotPositions.size > 0 || this.mapData.size > 0;
        if (placeholder) {
            placeholder.style.display = hasContent ? 'none' : 'flex';
        }

        // Add robot positions
        this.robotPositions.forEach((position, robotId) => {
            this.addRobotMarker(robotId, position.x, position.y);
        });
    }

    addRobotMarker(robotId, x, y) {
        const mapCanvas = this.element.querySelector('#map-canvas');
        if (!mapCanvas) return;

        const marker = document.createElement('div');
        marker.className = `robot-position ${robotId.replace('_', '-')}`;
        
        // Convert world coordinates to canvas coordinates
        const canvasCoords = this.convertMapToCanvas(x, y);
        
        marker.style.left = `${canvasCoords.x}px`;
        marker.style.top = `${canvasCoords.y}px`;
        marker.title = `${robotId}: (${x.toFixed(3)}, ${y.toFixed(3)})`;
        
        mapCanvas.appendChild(marker);
    }

    // CONTROL PAD MANAGEMENT
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

    // MAP MANAGEMENT METHODS
    clearMapData() {
        this.mapData.clear();
        this.currentMapInfo = null;
        
        // Remove map canvas
        const mapCanvas = this.element.querySelector('#map-canvas');
        const renderCanvas = mapCanvas?.querySelector('#map-render-canvas');
        if (renderCanvas) {
            renderCanvas.remove();
        }
        
        // Show placeholder
        const placeholder = this.element.querySelector('#map-placeholder');
        if (placeholder) {
            placeholder.style.display = 'flex';
        }
        
        this.updateMapVisualization();
    }

    getMapData() {
        return this.mapData;
    }

    setMapBounds(minBound, maxBound) {
        this.mapBounds = { min: minBound, max: maxBound };
        this.updateMapVisualization();
    }

    // Map saving functionality
    async saveMap(robotId, mapName) {
        try {
            // Use SLAM toolbox save service
            const mapSaver = new ROSLIB.Service({
                ros: window.rosConnector.connections.get(robotId),
                name: '/slam_toolbox/save_map',
                serviceType: 'slam_toolbox/SaveMap'
            });
            
            const request = new ROSLIB.ServiceRequest({
                name: { data: mapName || `map_${robotId}_${Date.now()}` }
            });
            
            mapSaver.callService(request, (result) => {
                this.log(`Map saved successfully for ${robotId}`, 'info');
            }, (error) => {
                this.log(`Failed to save map for ${robotId}: ${error}`, 'error');
            });
            
        } catch (error) {
            this.log(`Error saving map: ${error.message}`, 'error');
        }
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
