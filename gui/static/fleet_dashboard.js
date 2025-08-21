document.addEventListener('DOMContentLoaded', () => {
    let updateInterval = null;
    let systemStartTime = Date.now();
    let activityLog = [];

    // DOM elements
    const systemTimeSpan = document.getElementById('system-time');
    const robotList = document.getElementById('robot-list');
    const taskTableBody = document.getElementById('task-table-body');
    const activityLogDiv = document.getElementById('activity-log');
    const alertBar = document.getElementById('alert-bar');
    const alertMessage = document.getElementById('alert-message');

    // Stat elements
    const statTotalTasks = document.getElementById('stat-total-tasks');
    const statCompletedTasks = document.getElementById('stat-completed-tasks');
    const statPendingTasks = document.getElementById('stat-pending-tasks');
    const statActiveTasks = document.getElementById('stat-active-tasks');
    const statSuccessRate = document.getElementById('stat-success-rate');
    const statUptime = document.getElementById('stat-uptime');

    // Counter elements
    const robotCount = document.getElementById('robot-count');
    const taskCount = document.getElementById('task-count');
    const logCount = document.getElementById('log-count');

    // System time update
    function updateSystemTime() {
        const now = new Date();
        systemTimeSpan.textContent = now.toLocaleTimeString('ko-KR', { hour12: false });
        
        // Update uptime
        const uptimeMs = Date.now() - systemStartTime;
        const hours = Math.floor(uptimeMs / 3600000);
        const minutes = Math.floor((uptimeMs % 3600000) / 60000);
        const seconds = Math.floor((uptimeMs % 60000) / 1000);
        statUptime.textContent = `${hours.toString().padStart(2, '0')}:${minutes.toString().padStart(2, '0')}:${seconds.toString().padStart(2, '0')}`;
    }

    // Utility functions
    function formatDateTime(dateString) {
        if (!dateString) return 'N/A';
        const date = new Date(dateString);
        return date.toLocaleString('ko-KR', {
            month: '2-digit',
            day: '2-digit',
            hour: '2-digit',
            minute: '2-digit',
            second: '2-digit',
            hour12: false
        });
    }

    function formatElapsedTime(dateString) {
        if (!dateString) return 'N/A';
        const now = new Date();
        const created = new Date(dateString);
        const diffMs = now - created;
        
        const hours = Math.floor(diffMs / 3600000);
        const minutes = Math.floor((diffMs % 3600000) / 60000);
        const seconds = Math.floor((diffMs % 60000) / 1000);
        
        if (hours > 0) {
            return `${hours}h ${minutes}m`;
        } else if (minutes > 0) {
            return `${minutes}m ${seconds}s`;
        } else {
            return `${seconds}s`;
        }
    }

    function getStatusText(status) {
        const statusMap = {
            'idle': 'IDLE',
            'moving_to_arm': 'TO_ARM',
            'picking': 'PICKING',
            'moving_to_user': 'DELIVERY',
            'waiting_confirm': 'WAITING',
            'returning_to_dock': 'RETURNING',
            'offline': 'OFFLINE'
        };
        return statusMap[status] || status.toUpperCase();
    }

    function getStatusClass(status) {
        const statusClassMap = {
            'idle': 'idle',
            'moving_to_arm': 'busy',
            'picking': 'busy',
            'moving_to_user': 'busy',
            'waiting_confirm': 'busy',
            'returning_to_dock': 'busy',
            'offline': 'offline'
        };
        return statusClassMap[status] || 'offline';
    }

    function getTaskStatusText(status) {
        const statusMap = {
            '대기': 'PENDING',
            '할당': 'ASSIGNED',
            '이동중': 'MOVING',
            '집기중': 'PICKING',
            '수령대기': 'WAITING',
            '완료': 'COMPLETED',
            '실패': 'FAILED'
        };
        return statusMap[status] || status.toUpperCase();
    }

    function getTaskStatusClass(status) {
        const statusClassMap = {
            '대기': 'pending',
            '할당': 'assigned',
            '이동중': 'active',
            '집기중': 'active',
            '수령대기': 'active',
            '완료': 'completed',
            '실패': 'failed'
        };
        return statusClassMap[status] || 'pending';
    }

    function addActivityLogEntry(type, message) {
        const entry = {
            timestamp: new Date(),
            type: type,
            message: message
        };
        
        activityLog.unshift(entry);
        
        // Keep only last 50 entries
        if (activityLog.length > 50) {
            activityLog = activityLog.slice(0, 50);
        }
        
        renderActivityLog();
    }

    function showAlert(message, duration = 5000) {
        alertMessage.textContent = message;
        alertBar.classList.add('show');
        
        setTimeout(() => {
            alertBar.classList.remove('show');
        }, duration);
    }

    // Render functions
    function renderRobots(robots) {
        if (!robots || robots.length === 0) {
            robotList.innerHTML = '<div class="no-data">No robots connected</div>';
            robotCount.textContent = '0 Units';
            return;
        }

        let html = '';
        let onlineCount = 0;

        robots.forEach(robot => {
            const statusClass = getStatusClass(robot.status);
            const statusText = getStatusText(robot.status);
            
            if (robot.status !== 'offline') onlineCount++;

            const lastUpdate = robot.last_update ? new Date(robot.last_update * 1000).toLocaleTimeString('ko-KR', { hour12: false }) : 'N/A';
            const currentTask = robot.current_task ? `#${robot.current_task.task_id}` : 'None';
            
            // Battery information
            const batteryLevel = robot.battery || 0;
            const isRealTime = robot.battery_real_time || false;
            console.log(`DEBUG: Robot ${robot.id} - battery: ${batteryLevel}, real_time: ${isRealTime}`);
            let batteryIcon = '🟢';
            let batteryClass = 'success';
            
            if (batteryLevel <= 20) {
                batteryIcon = '🔴';
                batteryClass = 'danger';
            } else if (batteryLevel <= 40) {
                batteryIcon = '🟡';
                batteryClass = 'warning';
            }
            
            const batteryStatus = `${batteryIcon} ${batteryLevel}%${isRealTime ? ' ⚡' : ' 📁'}`;

            html += `
                <div class="robot-item">
                    <div class="robot-info">
                        <div class="robot-status-indicator ${statusClass}"></div>
                        <div>
                            <div class="robot-name">${robot.name}</div>
                            <div class="robot-details">ID: ${robot.id} | Task: ${currentTask} | Battery: ${batteryStatus} | Updated: ${lastUpdate}</div>
                        </div>
                    </div>
                    <div class="robot-status-text">${statusText}</div>
                </div>
            `;
        });

        robotList.innerHTML = html;
        robotCount.textContent = `${robots.length} Unit${robots.length > 1 ? 's' : ''} (${onlineCount} Online)`;
    }

    function renderTasks(allTasks) {
        const tasks = [...(allTasks.pending || []), ...(allTasks.active || []), ...(allTasks.completed || [])];
        
        if (!tasks || tasks.length === 0) {
            taskTableBody.innerHTML = '<tr><td colspan="7" class="no-data">No tasks available</td></tr>';
            taskCount.textContent = '0 Tasks';
            return;
        }

        let html = '';
        tasks.forEach(task => {
            const statusClass = getTaskStatusClass(task.status);
            const statusText = getTaskStatusText(task.status);
            const createdTime = formatDateTime(task.created_at);
            const elapsedTime = formatElapsedTime(task.created_at);
            const requesterName = task.requester_name || `User ${task.requester_resident_id}`;
            const robotName = task.bot_name || (task.assigned_bot_id ? `Robot ${task.assigned_bot_id}` : 'Unassigned');
            const taskType = task.task_type === '배달' ? 'DELIVERY' : 'CALL';

            html += `
                <tr>
                    <td><span class="task-id">#${task.task_id}</span></td>
                    <td>${taskType}</td>
                    <td><span class="task-status ${statusClass}">${statusText}</span></td>
                    <td>${requesterName}</td>
                    <td>${robotName}</td>
                    <td class="task-time">${createdTime}</td>
                    <td class="task-time">${elapsedTime}</td>
                </tr>
            `;
        });

        taskTableBody.innerHTML = html;
        taskCount.textContent = `${tasks.length} Task${tasks.length > 1 ? 's' : ''}`;
    }

    function renderActivityLog() {
        if (activityLog.length === 0) {
            activityLogDiv.innerHTML = '<div class="no-data">No system events</div>';
            logCount.textContent = '0 Events';
            return;
        }

        let html = '';
        activityLog.forEach(entry => {
            const timeStr = entry.timestamp.toLocaleTimeString('ko-KR', { hour12: false });
            html += `
                <div class="activity-item activity-type-${entry.type}">
                    <div class="activity-time">${timeStr}</div>
                    <div class="activity-message">${entry.message}</div>
                </div>
            `;
        });

        activityLogDiv.innerHTML = html;
        logCount.textContent = `${activityLog.length} Event${activityLog.length > 1 ? 's' : ''}`;
    }

    function updateStatistics(stats) {
        statTotalTasks.textContent = stats.total_tasks_today || 0;
        statCompletedTasks.textContent = stats.completed_tasks_today || 0;
        statPendingTasks.textContent = stats.pending_count || 0;
        statActiveTasks.textContent = stats.active_count || 0;
        statSuccessRate.textContent = `${stats.success_rate || 0}%`;
    }

    function showError(message) {
        const errorHtml = `<div class="error-state"><i class="bi bi-exclamation-triangle me-2"></i>CONNECTION ERROR: ${message}</div>`;
        robotList.innerHTML = errorHtml;
        taskTableBody.innerHTML = `<tr><td colspan="7" class="error-state">CONNECTION ERROR: ${message}</td></tr>`;
        activityLogDiv.innerHTML = errorHtml;
        
        addActivityLogEntry('error', `System error: ${message}`);
        showAlert(`SYSTEM ERROR: ${message}`);
    }

    // Fetch and update data
    async function fetchFleetStatus() {
        try {
            const response = await fetch('/api/fleet/status');
            
            if (!response.ok) {
                throw new Error(`HTTP ${response.status}: ${response.statusText}`);
            }
            
            const result = await response.json();

            if (result.success) {
                const data = result.data;
                
                // Update all sections
                renderRobots(data.robots);
                renderTasks(data.task_queue);
                updateStatistics(data.statistics);

                // Log system events
                if (typeof fetchFleetStatus.lastTaskCount === 'undefined') {
                    fetchFleetStatus.lastTaskCount = 0;
                }
                if (data.statistics.total_tasks_today > fetchFleetStatus.lastTaskCount) {
                    addActivityLogEntry('info', `New task created (Total: ${data.statistics.total_tasks_today})`);
                    fetchFleetStatus.lastTaskCount = data.statistics.total_tasks_today;
                }

                // Check for alerts
                if (data.statistics.pending_count > 5) {
                    addActivityLogEntry('warning', `High task queue: ${data.statistics.pending_count} pending tasks`);
                }

                data.robots.forEach(robot => {
                    if (robot.status === 'offline') {
                        addActivityLogEntry('error', `Robot ${robot.name} is offline`);
                    }
                });

            } else {
                throw new Error(result.error || 'Unknown API error');
            }
        } catch (error) {
            console.error('Failed to fetch fleet status:', error);
            showError(error.message);
        }
    }

    // Initialize dashboard
    async function initDashboard() {
        console.log('Fleet Management System initialized');
        
        // Add initial log entry
        addActivityLogEntry('success', 'Fleet Management System started');
        
        // Update system time immediately and every second
        updateSystemTime();
        setInterval(updateSystemTime, 1000);
        
        // Initial data fetch
        await fetchFleetStatus();

        // Set up auto-refresh every 2 seconds for real-time monitoring
        updateInterval = setInterval(fetchFleetStatus, 2000);
    }

    // Cleanup on page unload
    window.addEventListener('beforeunload', () => {
        if (updateInterval) {
            clearInterval(updateInterval);
        }
    });

    // Add keyboard shortcuts for power users
    document.addEventListener('keydown', (e) => {
        if (e.ctrlKey || e.metaKey) {
            switch (e.key) {
                case 'r':
                    e.preventDefault();
                    fetchFleetStatus();
                    addActivityLogEntry('info', 'Manual refresh triggered');
                    break;
                case '1':
                    e.preventDefault();
                    document.querySelector('.panel:nth-child(1)').scrollIntoView();
                    break;
                case '2':
                    e.preventDefault();
                    document.querySelector('.panel:nth-child(2)').scrollIntoView();
                    break;
                case '3':
                    e.preventDefault();
                    document.querySelector('.panel:nth-child(3)').scrollIntoView();
                    break;
            }
        }
    });

    // Start dashboard
    initDashboard();
});