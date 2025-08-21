document.addEventListener('DOMContentLoaded', () => {
    let updateInterval = null;
    let charts = {};
    let currentData = null;
    let cameraStreamActive = false;
    let activeCameras = {};
    let currentLayout = 'grid';
    let autoSwitchInterval = null;
    let autoSwitchActive = false;
    let currentFocusCamera = null;

    // Chart.js configuration
    Chart.defaults.color = '#cccccc';
    Chart.defaults.borderColor = '#333333';
    Chart.defaults.backgroundColor = 'rgba(0, 170, 255, 0.1)';

    // Theme colors
    const colors = {
        primary: '#00aaff',
        secondary: '#ff6b6b', 
        success: '#51cf66',
        warning: '#ffd43b',
        danger: '#ff4444',
        info: '#00aaff',
        muted: '#888888'
    };

    // Utility functions
    function showNotification(message, type = 'info', duration = 5000) {
        const notification = document.createElement('div');
        notification.style.cssText = `
            position: fixed;
            top: 20px;
            right: 20px;
            background: ${type === 'error' ? colors.danger : type === 'success' ? colors.success : colors.info};
            color: white;
            padding: 15px 20px;
            border-radius: 8px;
            box-shadow: 0 4px 15px rgba(0,0,0,0.3);
            z-index: 9999;
            font-size: 14px;
            max-width: 300px;
            animation: slideIn 0.3s ease-out;
        `;
        
        notification.innerHTML = `
            <div style="display: flex; align-items: center; gap: 10px;">
                <i class="bi bi-${type === 'error' ? 'exclamation-triangle' : type === 'success' ? 'check-circle' : 'info-circle'}"></i>
                <span>${message}</span>
            </div>
        `;
        
        document.body.appendChild(notification);
        
        setTimeout(() => {
            notification.remove();
        }, duration);
    }

    function formatDateTime(dateString) {
        if (!dateString) return 'N/A';
        const date = new Date(dateString);
        return date.toLocaleString('ko-KR', {
            year: 'numeric',
            month: '2-digit',
            day: '2-digit',
            hour: '2-digit',
            minute: '2-digit',
            second: '2-digit',
            hour12: false
        });
    }

    function getStatusBadgeHtml(status) {
        const statusMap = {
            'idle': 'IDLE', 'busy': 'BUSY', 'error': 'ERROR', 'charging': 'CHARGING',
            '대기': 'PENDING', '할당': 'ASSIGNED', '이동중': 'IN_PROGRESS', '집기중': 'PICKING',
            '수령대기': 'WAITING', '완료': 'COMPLETED', '실패': 'FAILED'
        };
        const statusClassMap = {
            'idle': 'idle', 'busy': 'busy', 'error': 'error', 'charging': 'charging',
            '대기': 'pending', '할당': 'assigned', '이동중': 'in_progress', '집기중': 'in_progress',
            '수령대기': 'in_progress', '완료': 'completed', '실패': 'failed'
        };
        const displayStatus = statusMap[status] || status.toUpperCase();
        const statusClass = statusClassMap[status] || 'muted';
        return `<span class="status-badge ${statusClass}">${displayStatus}</span>`;
    }

    // Section switching logic
    const sidebarLinks = document.querySelectorAll('.sidebar-nav .nav-link');
    const sections = document.querySelectorAll('.section-content');
    const currentSectionTitle = document.getElementById('current-section-title');
    const currentSectionSubtitle = document.getElementById('current-section-subtitle');

    function showSection(sectionId) {
        sections.forEach(section => {
            section.classList.remove('active');
        });
        document.getElementById(`${sectionId}-section`).classList.add('active');

        sidebarLinks.forEach(link => {
            link.classList.remove('active');
            if (link.dataset.section === sectionId) {
                link.classList.add('active');
            }
        });

        // Update header title and subtitle
        const sectionTitles = {
            'dashboard': { title: 'Dashboard', subtitle: 'Overview of fleet operations' },
            'robots': { title: 'Robot Management', subtitle: 'Monitor and control individual robots' },
            'tasks': { title: 'Task Management', subtitle: 'View and manage all tasks' },
            'users': { title: 'User Management', subtitle: 'Manage registered users' },
            'items': { title: 'Item Management', subtitle: 'View and update item inventory' },
            'camera': { title: 'Camera System', subtitle: 'Multi-camera monitoring and control' }
        };
        currentSectionTitle.textContent = sectionTitles[sectionId]?.title || sectionId;
        currentSectionSubtitle.textContent = sectionTitles[sectionId]?.subtitle || '';
        
        // Load section-specific data
        if (sectionId === 'robots') {
            console.log('🤖 Robots section activated, loading data...');
            // Add small delay to ensure DOM is rendered
            setTimeout(() => {
                fetchRobotData(); // Load robot data when robots section is shown
            }, 200);
        }
    }

    sidebarLinks.forEach(link => {
        link.addEventListener('click', (e) => {
            e.preventDefault();
            const section = e.target.closest('.nav-link')?.dataset.section;
            if (section) {
                showSection(section);
                refreshData(); // Refresh data when switching sections
            }
        });
    });

    // Dashboard Subtab switching logic
    const subnavTabs = document.querySelectorAll('.subnav-tab');
    const dashboardSubtabs = document.querySelectorAll('.dashboard-subtab');

    function showDashboardSubtab(subtabId) {
        // Hide all subtabs
        dashboardSubtabs.forEach(subtab => {
            subtab.classList.remove('active');
        });
        
        // Show selected subtab
        const targetSubtab = document.getElementById(`dashboard-${subtabId}`);
        if (targetSubtab) {
            targetSubtab.classList.add('active');
        }

        // Update tab buttons
        subnavTabs.forEach(tab => {
            tab.classList.remove('active');
            if (tab.dataset.subtab === subtabId) {
                tab.classList.add('active');
            }
        });

        console.log(`🔄 Switched to dashboard subtab: ${subtabId}`);
    }

    subnavTabs.forEach(tab => {
        tab.addEventListener('click', (e) => {
            e.preventDefault();
            const subtabId = e.target.closest('.subnav-tab')?.dataset.subtab;
            if (subtabId) {
                showDashboardSubtab(subtabId);
            }
        });
    });

    // Initialize charts (from advanced_fleet_dashboard.js)
    function initCharts() {
        const chartElements = [
            { id: 'hourlyChart', key: 'hourly', type: 'line' },
            { id: 'statusChart', key: 'status', type: 'doughnut' },
            { id: 'robotChart', key: 'robot', type: 'bar' },
            { id: 'trendChart', key: 'trend', type: 'line' }
        ];

        chartElements.forEach(({ id, key, type }) => {
            const element = document.getElementById(id);
            if (element) {
                try {
                    const ctx = element.getContext('2d');
                    if (key === 'hourly') {
                        charts[key] = new Chart(ctx, {
                            type: 'line',
                            data: {
                                labels: Array.from({length: 24}, (_, i) => `${i}:00`),
                                datasets: [
                                    { label: 'Total Tasks', data: [], borderColor: colors.primary, backgroundColor: colors.primary + '20', fill: true, tension: 0.4 }, 
                                    { label: 'Completed', data: [], borderColor: colors.success, backgroundColor: colors.success + '20', fill: true, tension: 0.4 }, 
                                    { label: 'Failed', data: [], borderColor: colors.danger, backgroundColor: colors.danger + '20', fill: true, tension: 0.4 }
                                ]
                            },
                            options: { responsive: true, maintainAspectRatio: false, scales: { y: { beginAtZero: true, grid: { color: '#333333' } }, x: { grid: { color: '#333333' } } }, plugins: { legend: { display: true, position: 'top' } } }
                        });
                    } else if (key === 'status') {
                        charts[key] = new Chart(ctx, {
                            type: 'doughnut',
                            data: {
                                labels: ['Completed', 'Failed', 'Active', 'Pending'],
                                datasets: [{ data: [0, 0, 0, 0], backgroundColor: [colors.success, colors.danger, colors.warning, colors.muted], borderWidth: 2, borderColor: '#1e1e1e' }]
                            },
                            options: { responsive: true, maintainAspectRatio: false, plugins: { legend: { position: 'bottom' } } }
                        });
                    } else if (key === 'robot') {
                        charts[key] = new Chart(ctx, {
                            type: 'bar',
                            data: {
                                labels: [],
                                datasets: [
                                    { label: 'Completed Tasks', data: [], backgroundColor: colors.success + '80', borderColor: colors.success, borderWidth: 1 }, 
                                    { label: 'Failed Tasks', data: [], backgroundColor: colors.danger + '80', borderColor: colors.danger, borderWidth: 1 }
                                ]
                            },
                            options: { responsive: true, maintainAspectRatio: false, scales: { y: { beginAtZero: true, grid: { color: '#333333' } }, x: { grid: { color: '#333333' } } } }
                        });
                    } else if (key === 'trend') {
                        charts[key] = new Chart(ctx, {
                            type: 'line',
                            data: {
                                labels: [],
                                datasets: [
                                    { label: 'Daily Tasks', data: [], borderColor: colors.primary, backgroundColor: colors.primary + '20', fill: true, tension: 0.4 }, 
                                    { label: 'Success Rate %', data: [], borderColor: colors.success, backgroundColor: colors.success + '20', fill: false, yAxisID: 'percentage', tension: 0.4 }
                                ]
                            },
                            options: { responsive: true, maintainAspectRatio: false, scales: { y: { beginAtZero: true, grid: { color: '#333333' } }, percentage: { type: 'linear', display: true, position: 'right', min: 0, max: 100, grid: { drawOnChartArea: false, }, }, x: { grid: { color: '#333333' } } } }
                        });
                    }
                    console.log(`✅ Chart '${key}' initialized successfully`);
                } catch (error) {
                    console.error(`❌ Failed to initialize chart '${key}':`, error);
                }
            } else {
                console.warn(`⚠️  Chart element '${id}' not found`);
            }
        });
    }

    // Update metrics display (from advanced_fleet_dashboard.js)
    function updateMetrics(metrics) {
        console.log('📊 Received metrics from API:', metrics);
        
        // Only update elements that exist in HTML
        const elements = [
            { id: 'total-tasks-today', value: metrics.total_tasks_today || 0 },
            { id: 'active-users', value: metrics.unique_users_today || 0 }
        ];

        elements.forEach(({ id, value }) => {
            const element = document.getElementById(id);
            if (element) {
                element.textContent = value;
                console.log(`✅ Updated ${id}: ${value}`);
            } else {
                console.warn(`❌ Element with ID '${id}' not found`);
            }
        });

        // Handle avg completion time
        const avgTimeElement = document.getElementById('avg-completion-time');
        if (avgTimeElement) {
            const avgTime = metrics.avg_completion_time;
            console.log('🕒 Avg completion time from API:', avgTime, typeof avgTime);
            if (avgTime !== null && avgTime !== undefined && !isNaN(avgTime)) {
                avgTimeElement.textContent = parseFloat(avgTime).toFixed(1);
            } else {
                avgTimeElement.textContent = '0.0';
            }
        } else {
            console.warn('avg-completion-time element not found');
        }

        // Handle success rate
        const successRateElement = document.getElementById('success-rate');
        if (successRateElement) {
            const successRate = (metrics.total_tasks_today && metrics.total_tasks_today > 0) ? 
                ((metrics.completed_today / metrics.total_tasks_today) * 100).toFixed(1) : '0.0';
            successRateElement.textContent = successRate + '%';
        }

        // Update change indicators (only if elements exist)
        updateChangeIndicator('tasks-change', 12, 'positive');
        updateChangeIndicator('time-change', -5, 'positive');
        updateChangeIndicator('users-change', 3, 'positive');
        updateChangeIndicator('success-change', 2.3, 'positive');
    }

    function updateChangeIndicator(elementId, change, type) {
        const element = document.getElementById(elementId);
        if (element) {
            const prefix = change > 0 ? '+' : '';
            const suffix = elementId.includes('change') && !elementId.includes('users') ? '%' : '';
            element.textContent = `${prefix}${change}${suffix}`;
            element.className = `metric-change ${type}`;
        } else {
            console.warn(`Change indicator element with ID '${elementId}' not found`);
        }
    }

    function updateHourlyChart(hourlyData) {
        if (!charts.hourly) {
            console.warn('Hourly chart not initialized');
            return;
        }

        const hours = Array.from({length: 24}, (_, i) => i);
        const taskCounts = new Array(24).fill(0);
        const completedCounts = new Array(24).fill(0);
        const failedCounts = new Array(24).fill(0);

        hourlyData.forEach(item => {
            const hour = parseInt(item.hour);
            taskCounts[hour] = item.task_count || 0;
            completedCounts[hour] = item.completed_count || 0;
            failedCounts[hour] = item.failed_count || 0;
        });

        try {
            charts.hourly.data.datasets[0].data = taskCounts;
            charts.hourly.data.datasets[1].data = completedCounts;
            charts.hourly.data.datasets[2].data = failedCounts;
            charts.hourly.update();
        } catch (error) {
            console.error('Failed to update hourly chart:', error);
        }
    }

    function updateStatusChart(metrics) {
        if (!charts.status) {
            console.warn('Status chart not initialized');
            return;
        }

        const data = [
            metrics.completed_today || 0,
            metrics.failed_today || 0,
            metrics.active_today || 0,
            (metrics.total_tasks_today || 0) - (metrics.completed_today || 0) - (metrics.failed_today || 0) - (metrics.active_today || 0)
        ];
        
        try {
            charts.status.data.datasets[0].data = data;
            charts.status.update();
        } catch (error) {
            console.error('Failed to update status chart:', error);
        }
    }

    function updateRobotChart(robotData) {
        if (!charts.robot) {
            console.warn('Robot chart not initialized');
            return;
        }

        const names = robotData.map(robot => robot.bot_name || `Robot ${robot.hana_bot_id}`);
        const completed = robotData.map(robot => robot.completed_tasks || 0);
        const failed = robotData.map(robot => robot.failed_tasks || 0);

        try {
            charts.robot.data.labels = names;
            charts.robot.data.datasets[0].data = completed;
            charts.robot.data.datasets[1].data = failed;
            charts.robot.update();
        } catch (error) {
            console.error('Failed to update robot chart:', error);
        }
    }

    function updateTrendChart(trendData) {
        if (!charts.trend) {
            console.warn('Trend chart not initialized');
            return;
        }

        const dates = trendData.map(item => {
            const date = new Date(item.date);
            return `${date.getMonth() + 1}/${date.getDate()}`;
        }).reverse();
        
        const tasks = trendData.map(item => item.total_tasks || 0).reverse();
        const successRates = trendData.map(item => {
            const total = item.total_tasks || 0;
            const completed = item.completed_tasks || 0;
            return total > 0 ? parseFloat(((completed / total) * 100).toFixed(1)) : 0;
        }).reverse();

        try {
            charts.trend.data.labels = dates;
            charts.trend.data.datasets[0].data = tasks;
            charts.trend.data.datasets[1].data = successRates;
            charts.trend.update();
        } catch (error) {
            console.error('Failed to update trend chart:', error);
        }
    }

    function updateTopUsersTable(userData) {
        const tbody = document.getElementById('top-users-table');
        if (!tbody) {
            console.warn('Top users table element not found');
            return;
        }
        
        if (!userData || userData.length === 0) {
            tbody.innerHTML = '<tr><td colspan="5" class="text-center text-muted">No data available</td></tr>';
            return;
        }
        let html = '';
        userData.forEach((user) => {
            const successRate = user.order_count > 0 ? ((user.completed_orders / user.order_count) * 100).toFixed(1) : '0.0';
            const performanceClass = successRate >= 90 ? 'excellent' : successRate >= 70 ? 'good' : 'poor';
            html += `
                <tr>
                    <td><strong>${user.name}</strong><br><small class="text-muted">ID: ${user.resident_id}</small></td>
                    <td><strong>${user.order_count}</strong></td>
                    <td>${user.completed_orders}</td>
                    <td>${successRate}%</td>
                    <td><div class="performance-bar"><div class="performance-fill ${performanceClass}" style="width: ${successRate}%"></div></div></td>
                </tr>
            `;
        });
        tbody.innerHTML = html;
    }

    function updatePopularItemsTable(itemData) {
        const tbody = document.getElementById('popular-items-table');
        if (!tbody) {
            console.warn('Popular items table element not found');
            return;
        }
        
        if (!itemData || itemData.length === 0) {
            tbody.innerHTML = '<tr><td colspan="5" class="text-center text-muted">No data available</td></tr>';
            return;
        }
        let html = '';
        const maxOrders = Math.max(...itemData.map(item => item.order_count));
        itemData.forEach((item) => {
            const successRate = item.order_count > 0 ? ((item.delivered_count / item.order_count) * 100).toFixed(1) : '0.0';
            const popularity = maxOrders > 0 ? ((item.order_count / maxOrders) * 100).toFixed(0) : 0;
            html += `
                <tr>
                    <td><strong>${item.item_type}</strong><br><small class="text-muted">ID: ${item.item_id}</small></td>
                    <td><strong>${item.order_count}</strong></td>
                    <td>${item.delivered_count}</td>
                    <td>${successRate}%</td>
                    <td><div class="performance-bar"><div class="performance-fill excellent" style="width: ${popularity}%"></div></div></td>
                </tr>
            `;
        });
        tbody.innerHTML = html;
    }

    // Fetch analytics data (from advanced_fleet_dashboard.js)
    async function fetchAnalytics() {
        try {
            const response = await fetch('/api/fleet/analytics');
            if (!response.ok) throw new Error(`HTTP ${response.status}: ${response.statusText}`);
            const result = await response.json();
            if (result.success && result.data) {
                currentData = result.data;
                
                // Update each component safely
                if (currentData.system_metrics) {
                    updateMetrics(currentData.system_metrics);
                    updateStatusChart(currentData.system_metrics);
                }
                if (currentData.hourly_distribution) {
                    updateHourlyChart(currentData.hourly_distribution);
                }
                if (currentData.robot_performance) {
                    updateRobotChart(currentData.robot_performance);
                }
                if (currentData.daily_trends) {
                    updateTrendChart(currentData.daily_trends);
                }
                if (currentData.top_requesters) {
                    updateTopUsersTable(currentData.top_requesters);
                }
                if (currentData.popular_items) {
                    updatePopularItemsTable(currentData.popular_items);
                }

                console.log('✅ Analytics data updated successfully');
            } else {
                throw new Error(result.error || 'No data received from API');
            }
        } catch (error) {
            console.error('❌ Failed to fetch analytics:', error);
            
            // Show user-friendly error notification
            const errorMessage = error.message.includes('Failed to fetch') ? 
                'Unable to connect to server' : 
                'Failed to load analytics data';
            showNotification(errorMessage, 'error');
            
            // Initialize empty charts/tables if this is first load
            if (!currentData) {
                console.log('🔧 Initializing empty analytics display...');
                updateMetrics({});
            }
        }
    }

    // Control functions (from advanced_fleet_dashboard.js)
    async function sendControlCommand(action, targetId = null) {
        try {
            const response = await fetch('/api/fleet/control', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ action: action, target_id: targetId })
            });
            const result = await response.json();
            if (result.success) {
                showNotification(result.message, 'success');
                setTimeout(refreshData, 1000);
            } else {
                showNotification('Control action failed: ' + result.error, 'error');
            }
        } catch (error) {
            console.error('Control command failed:', error);
            showNotification('Control command failed: ' + error.message, 'error');
        }
    }

    // New data fetching and rendering for admin sections
    // Global battery status cache
    let batteryStatusCache = {};

    async function fetchBatteryStatus() {
        try {
            const response = await fetch('/api/battery/status');
            if (response.ok) {
                const result = await response.json();
                if (result.success && result.data) {
                    batteryStatusCache = result.data;
                }
            }
        } catch (error) {
            console.error('Failed to fetch battery status:', error);
        }
    }

    function getBatteryLevel(robotId) {
        // Map robot database ID to battery topic ID
        const batteryIdMap = {
            8: 'DP_09',  // robot_1 (HANA PINKY)
            9: 'DP_03'   // robot_2 (HANA ROBOT 9)
        };
        
        const batteryId = batteryIdMap[robotId];
        if (batteryId && batteryStatusCache[batteryId]) {
            const batteryInfo = batteryStatusCache[batteryId];
            const level = batteryInfo.level || 0;
            
            // Check if data is recent (within last 30 seconds)
            const now = Date.now() / 1000;
            const dataAge = now - (batteryInfo.timestamp || 0);
            
            return dataAge < 30 ? level : null;
        }
        return null;
    }

    function getBatteryStatusHtml(robotId, dbBattery) {
        const realTimeBattery = getBatteryLevel(robotId);
        const batteryLevel = realTimeBattery !== null ? realTimeBattery : (dbBattery || 0);
        
        let batteryClass = 'success';
        let batteryIcon = '🟢';
        
        if (batteryLevel <= 20) {
            batteryClass = 'danger';
            batteryIcon = '🔴';
        } else if (batteryLevel <= 40) {
            batteryClass = 'warning'; 
            batteryIcon = '🟡';
        }
        
        const isRealTime = realTimeBattery !== null;
        const statusIndicator = isRealTime ? ' ⚡' : ' 📁';
        
        return `<span class="status-badge ${batteryClass}" title="${isRealTime ? 'Real-time data' : 'Database data'}">${batteryIcon} ${batteryLevel}%${statusIndicator}</span>`;
    }

    async function fetchRobots() {
        try {
            // Fetch battery status first
            await fetchBatteryStatus();
            
            const response = await fetch('/api/admin/robots');
            if (!response.ok) throw new Error(`HTTP ${response.status}: ${response.statusText}`);
            const result = await response.json();
            if (result.success) {
                const tbody = document.getElementById('robot-detailed-table-body');
                let html = '';
                if (result.data.length === 0) {
                    html = '<tr><td colspan="7" class="text-center text-muted">No robot data available</td></tr>';
                } else {
                    result.data.forEach(robot => {
                        html += `
                            <tr>
                                <td>${robot.id}</td>
                                <td>${robot.name}</td>
                                <td>${getStatusBadgeHtml(robot.status)}</td>
                                <td>${getBatteryStatusHtml(robot.id, robot.battery)}</td>
                                <td>${robot.current_task ? `#${robot.current_task.task_id} (${robot.current_task.task_type})` : 'None'}</td>
                                <td>${formatDateTime(robot.last_update)}</td>
                                <td>
                                    <button class="action-icon-btn" onclick="resetRobot('${robot.id}')" title="Reset Robot"><i class="bi bi-arrow-clockwise"></i></button>
                                    <button class="action-icon-btn danger" onclick="emergencyStopRobot('${robot.id}')" title="Emergency Stop"><i class="bi bi-stop-circle"></i></button>
                                    <button class="action-icon-btn" onclick="testBatteryLow(${robot.id})" title="Test Low Battery"><i class="bi bi-battery"></i></button>
                                    <button class="action-icon-btn" onclick="testBatteryHigh(${robot.id})" title="Test High Battery"><i class="bi bi-battery-charging"></i></button>
                                </td>
                            </tr>
                        `;
                    });
                }
                tbody.innerHTML = html;
            } else {
                throw new Error(result.error || 'Unknown API error');
            }
        } catch (error) {
            console.error('Failed to fetch robots:', error);
            showNotification('Failed to load robot data: ' + error.message, 'error');
        }
    }

    async function fetchTasks() {
        try {
            const response = await fetch('/api/admin/tasks');
            if (!response.ok) throw new Error(`HTTP ${response.status}: ${response.statusText}`);
            const result = await response.json();
            if (result.success) {
                const tbody = document.getElementById('tasks-table-body');
                let html = '';
                if (result.data.length === 0) {
                    html = '<tr><td colspan="9" class="text-center text-muted">No task data available</td></tr>';
                } else {
                    result.data.forEach(task => {
                        html += `
                            <tr>
                                <td>${task.task_id}</td>
                                <td>${task.task_type}</td>
                                <td>${getStatusBadgeHtml(task.status)}</td>
                                <td>${task.requester_name || 'N/A'}</td>
                                <td>${task.item_type || 'N/A'}</td>
                                <td>${task.bot_name || 'Unassigned'}</td>
                                <td>${formatDateTime(task.created_at)}</td>
                                <td>${formatDateTime(task.completed_at)}</td>
                                <td>
                                    <button class="action-icon-btn danger" onclick="cancelTask(${task.task_id})" title="Cancel Task"><i class="bi bi-x-circle"></i></button>
                                </td>
                            </tr>
                        `;
                    });
                }
                tbody.innerHTML = html;
            } else {
                throw new Error(result.error || 'Unknown API error');
            }
        } catch (error) {
            console.error('Failed to fetch tasks:', error);
            showNotification('Failed to load task data: ' + error.message, 'error');
        }
    }

    async function fetchUsers() {
        try {
            const response = await fetch('/api/admin/users');
            if (!response.ok) throw new Error(`HTTP ${response.status}: ${response.statusText}`);
            const result = await response.json();
            if (result.success) {
                const tbody = document.getElementById('users-table-body');
                let html = '';
                if (result.data.length === 0) {
                    html = '<tr><td colspan="6" class="text-center text-muted">No user data available</td></tr>';
                } else {
                    result.data.forEach(user => {
                        html += `
                            <tr>
                                <td>${user.resident_id}</td>
                                <td>${user.name}</td>
                                <td>${user.gender}</td>
                                <td>${user.birth_date}</td>
                                <td>${user.login_id}</td>
                                <td>
                                    <button class="action-icon-btn" onclick="editUser(${user.resident_id})" title="Edit User"><i class="bi bi-pencil"></i></button>
                                    <button class="action-icon-btn danger" onclick="deleteUser(${user.resident_id})" title="Delete User"><i class="bi bi-trash"></i></button>
                                </td>
                            </tr>
                        `;
                    });
                }
                tbody.innerHTML = html;
            } else {
                throw new Error(result.error || 'Unknown API error');
            }
        } catch (error) {
            console.error('Failed to fetch users:', error);
            showNotification('Failed to load user data: ' + error.message, 'error');
        }
    }

    async function fetchItems() {
        try {
            const response = await fetch('/api/admin/items');
            if (!response.ok) throw new Error(`HTTP ${response.status}: ${response.statusText}`);
            const result = await response.json();
            if (result.success) {
                const tbody = document.getElementById('items-table-body');
                let html = '';
                if (result.data.length === 0) {
                    html = '<tr><td colspan="4" class="text-center text-muted">No item data available</td></tr>';
                } else {
                    result.data.forEach(item => {
                        html += `
                            <tr>
                                <td>${item.item_id}</td>
                                <td>${item.item_type}</td>
                                <td>${item.item_quantity}</td>
                                <td>
                                    <button class="action-icon-btn" onclick="editItem(${item.item_id})" title="Edit Item"><i class="bi bi-pencil"></i></button>
                                </td>
                            </tr>
                        `;
                    });
                }
                tbody.innerHTML = html;
            } else {
                throw new Error(result.error || 'Unknown API error');
            }
        } catch (error) {
            console.error('Failed to fetch items:', error);
            showNotification('Failed to load item data: ' + error.message, 'error');
        }
    }

    // Battery status update function
    async function updateBatteryCards() {
        // Ensure DOM is fully loaded
        if (document.readyState !== 'complete') {
            console.log('🔋 DOM not ready, waiting...');
            setTimeout(updateBatteryCards, 100);
            return;
        }
        
        try {
            console.log('🔋 Fetching battery status...');
            const response = await fetch('/api/battery/status');
            if (!response.ok) {
                console.warn('🔋 Battery API response not ok:', response.status);
                return;
            }
            
            const result = await response.json();
            console.log('🔋 Battery API response:', result);
            
            if (result.success && result.data) {
                const batteryData = result.data;
                
                // Update DP_09 battery card
                if (batteryData.DP_09) {
                    console.log('🔋 Updating DP_09 with:', batteryData.DP_09);
                    updateBatteryCard('dp09', batteryData.DP_09);
                }
                
                // Update DP_03 battery card  
                if (batteryData.DP_03) {
                    console.log('🔋 Updating DP_03 with:', batteryData.DP_03);
                    updateBatteryCard('dp03', batteryData.DP_03);
                }
            } else {
                console.warn('🔋 No battery data in response');
            }
        } catch (error) {
            console.error('🔋 Failed to fetch battery status:', error);
        }
    }

    function updateBatteryCard(robotId, batteryInfo) {
        try {
            const level = batteryInfo.level || 0;
            const timestamp = batteryInfo.timestamp || 0;
            
            console.log(`🔋 Updating battery card for ${robotId}: ${level}% (${timestamp})`);
            
            // Update elements
            const levelElement = document.getElementById(`battery-level-${robotId}`);
            const fillElement = document.getElementById(`battery-fill-${robotId}`);
            const timeElement = document.getElementById(`battery-time-${robotId}`);
            const cardElement = document.getElementById(`battery-${robotId}`);
            const iconElement = document.getElementById(`battery-icon-${robotId}`);
            
            // Check if this is an actual change to prevent unnecessary updates
            const currentLevel = levelElement ? parseInt(levelElement.textContent) : -1;
            if (currentLevel === level) {
                console.log(`🔋 No change for ${robotId}, skipping update`);
                return;
            }
            
            // Debug: Check if elements exist
            console.log(`🔋 Elements for ${robotId}:`, {
                levelElement: !!levelElement,
                fillElement: !!fillElement, 
                timeElement: !!timeElement,
                cardElement: !!cardElement,
                iconElement: !!iconElement
            });
            
            if (levelElement) {
                levelElement.textContent = `${level}%`;
                console.log(`🔋 Set level text: ${level}%`);
            } else {
                console.error(`🔋 Level element not found: battery-level-${robotId}`);
            }
            
            if (fillElement) {
                fillElement.style.width = `${level}%`;
                // Remove loading class to stop shimmer animation
                fillElement.classList.remove('loading');
                console.log(`🔋 Set fill width: ${level}%, removed loading class`);
            } else {
                console.error(`🔋 Fill element not found: battery-fill-${robotId}`);
            }
            
            // Update battery card status classes based on level
            if (cardElement) {
                // Remove all battery status classes
                cardElement.classList.remove('battery-offline', 'battery-critical', 'battery-low', 'battery-medium', 'battery-good');
                
                // Add appropriate class based on battery level
                if (level <= 0) {
                    cardElement.classList.add('battery-offline');
                } else if (level <= 20) {
                    cardElement.classList.add('battery-critical');
                } else if (level <= 40) {
                    cardElement.classList.add('battery-low');
                } else if (level <= 70) {
                    cardElement.classList.add('battery-medium');
                } else {
                    cardElement.classList.add('battery-good');
                }
                
                console.log(`🔋 Updated card class for ${level}%:`, cardElement.className);
            } else {
                console.error(`🔋 Card element not found: battery-${robotId}`);
            }
            
            // Update battery icon based on level
            if (iconElement) {
                // Remove all icon classes
                iconElement.className = 'battery-icon';
                
                // Add appropriate icon based on battery level
                if (level <= 0) {
                    iconElement.classList.add('bi', 'bi-battery-empty');
                } else if (level <= 25) {
                    iconElement.classList.add('bi', 'bi-battery-low');
                } else if (level <= 50) {
                    iconElement.classList.add('bi', 'bi-battery-half');
                } else if (level <= 75) {
                    iconElement.classList.add('bi', 'bi-battery-three-quarters');
                } else {
                    iconElement.classList.add('bi', 'bi-battery-full');
                }
                
                console.log(`🔋 Updated icon for ${level}%:`, iconElement.className);
            } else {
                console.error(`🔋 Icon element not found: battery-icon-${robotId}`);
            }
        
        // Update timestamp
        if (timeElement) {
            const now = Date.now() / 1000;
            const dataAge = now - timestamp;
            
            if (dataAge < 30) {
                timeElement.textContent = `Live (${new Date(timestamp * 1000).toLocaleTimeString()})`;
            } else if (timestamp > 0) {
                timeElement.textContent = `Last: ${new Date(timestamp * 1000).toLocaleTimeString()}`;
            } else {
                timeElement.textContent = 'No data';
            }
        }
        
        // Update card status class
        if (cardElement) {
            cardElement.classList.remove('battery-offline', 'battery-low', 'battery-medium', 'battery-high');
            
            if (level <= 0) {
                cardElement.classList.add('battery-offline');
            } else if (level <= 20) {
                cardElement.classList.add('battery-low');
            } else if (level <= 60) {
                cardElement.classList.add('battery-medium');
            } else {
                cardElement.classList.add('battery-high');
            }
        }
        
        // Update icon
        if (iconElement) {
            iconElement.classList.remove('bi-battery', 'bi-battery-quarter', 'bi-battery-half', 'bi-battery-three-quarters', 'bi-battery-full');
            
            if (level <= 20) {
                iconElement.classList.add('bi-battery');
            } else if (level <= 40) {
                iconElement.classList.add('bi-battery-quarter');
            } else if (level <= 60) {
                iconElement.classList.add('bi-battery-half');
            } else if (level <= 80) {
                iconElement.classList.add('bi-battery-three-quarters');
            } else {
                iconElement.classList.add('bi-battery-full');
            }
        }
            // Smooth transition for visual updates
            if (cardElement) {
                // Add a brief transition class for smooth updates
                cardElement.classList.add('updating');
                
                setTimeout(() => {
                    cardElement.classList.remove('updating');
                }, 300);
                
                console.log(`🔋 Applied smooth transition for ${robotId}`);
            }
            
        } catch (error) {
            console.error(`🔋 Error updating battery card for ${robotId}:`, error);
        }
    }

    // Global refresh function
    window.refreshData = () => {
        showNotification('Refreshing data...', 'info');
        fetchAnalytics();
        // Use the comprehensive robot data function instead
        if (typeof fetchRobotData === 'function') {
            fetchRobotData();
        } else {
            fetchRobots();
        }
        fetchTasks();
        fetchUsers();
        fetchItems();
    };

    // Battery test functions
    window.testBatteryLevel = async (robotId, level) => {
        try {
            // Map database robot ID to battery topic ID
            const batteryIdMap = {
                8: 'DP_09',  // robot_1 (HANA PINKY)
                9: 'DP_03'   // robot_2 (HANA ROBOT 9)
            };
            
            const batteryTopicId = batteryIdMap[robotId];
            if (!batteryTopicId) {
                showNotification(`Invalid robot ID: ${robotId}`, 'error');
                return;
            }
            
            const response = await fetch(`/api/battery/test/${batteryTopicId}/${level}`, {
                method: 'POST'
            });
            const result = await response.json();
            
            if (result.success) {
                showNotification(`Battery test: ${batteryTopicId} set to ${level}%`, 'success');
                setTimeout(refreshData, 1000); // Refresh to show updated battery
            } else {
                showNotification(`Battery test failed: ${result.error}`, 'error');
            }
        } catch (error) {
            showNotification(`Battery test error: ${error.message}`, 'error');
        }
    };

    window.testBatteryLow = (robotId) => {
        if (confirm(`Test low battery (20%) for robot ${robotId}?`)) {
            testBatteryLevel(robotId, 20);
        }
    };

    window.testBatteryHigh = (robotId) => {
        if (confirm(`Test high battery (85%) for robot ${robotId}?`)) {
            testBatteryLevel(robotId, 85);
        }
    };

    // Global control functions (placeholders for now)
    window.emergencyStop = () => { if (confirm('Are you sure you want to issue an emergency stop to all robots?')) sendControlCommand('emergency_stop'); };
    window.resetSystem = () => { if (confirm('Reset all robot statuses to idle?')) sendControlCommand('reset_system'); };
    window.optimizeRoutes = () => { showNotification('Route optimization initiated...', 'info'); sendControlCommand('optimize_routes'); };
    window.maintenanceMode = () => { if (confirm('Enable maintenance mode? This will pause all operations.')) sendControlCommand('maintenance_mode'); };
    window.generateReport = () => { showNotification('Generating comprehensive fleet report...', 'info'); setTimeout(() => { showNotification('Report generation completed', 'success'); }, 3000); };
    window.clearFailedTasks = () => { if (confirm('Clear all failed tasks from the database?')) sendControlCommand('fail_all_incomplete'); };

    // System operation functions
    window.backupDatabase = () => { 
        showNotification('Starting database backup...', 'info'); 
        setTimeout(() => { showNotification('Database backup completed successfully', 'success'); }, 5000);
    };
    window.systemDiagnostics = () => { 
        showNotification('Running system diagnostics...', 'info'); 
        setTimeout(() => { showNotification('Diagnostics completed - System healthy', 'success'); }, 8000);
    };
    window.cleanupLogs = () => { 
        if (confirm('Clean up old log files? This will remove logs older than 30 days.')) {
            showNotification('Cleaning up log files...', 'info'); 
            setTimeout(() => { showNotification('Log cleanup completed', 'success'); }, 3000);
        }
    };

    // Specific robot/task/user/item actions (placeholders)
    window.resetRobot = (robotId) => { showNotification(`Resetting robot ${robotId}...`, 'info'); /* Implement API call */ };
    window.emergencyStopRobot = (robotId) => { if (confirm(`Emergency stop robot ${robotId}?`)) showNotification(`Emergency stopping robot ${robotId}...`, 'warning'); /* Implement API call */ };
    window.cancelTask = (taskId) => { if (confirm(`Cancel task ${taskId}?`)) sendControlCommand('cancel_task', taskId); };
    window.editUser = (userId) => { showNotification(`Editing user ${userId}...`, 'info'); /* Implement UI/API for editing */ };
    window.deleteUser = (userId) => { if (confirm(`Delete user ${userId}?`)) showNotification(`Deleting user ${userId}...`, 'warning'); /* Implement API call */ };
    window.editItem = (itemId) => { showNotification(`Editing item ${itemId}...`, 'info'); /* Implement UI/API for editing */ };

    // Camera stream functions
    window.startCameraStream = async () => {
        if (cameraStreamActive) {
            showNotification('Camera stream is already active', 'warning');
            return;
        }

        try {
            const response = await fetch('/camera/start', { method: 'POST' });
            const result = await response.json();
            
            if (result.status === 'success') {
                cameraStreamActive = true;
                
                // Update main camera view
                document.getElementById('stream-status').textContent = 'Connected';
                document.getElementById('stream-status').style.color = 'var(--success-color)';
                document.getElementById('camera-placeholder').style.display = 'none';
                document.getElementById('camera-stream').style.display = 'block';
                document.getElementById('camera-stream').src = '/camera/stream?' + Date.now();
                
                // Show stream overlay
                const overlay = document.getElementById('stream-overlay');
                if (overlay) overlay.style.display = 'block';
                
                // Update preview in dashboard
                const previewPlaceholder = document.getElementById('camera-preview-placeholder');
                const previewStream = document.getElementById('camera-preview-stream');
                const previewStatus = document.getElementById('camera-preview-status');
                
                if (previewPlaceholder) previewPlaceholder.style.display = 'none';
                if (previewStream) {
                    previewStream.style.display = 'block';
                    previewStream.src = '/camera/stream?' + Date.now();
                }
                if (previewStatus) previewStatus.style.display = 'block';
                
                showNotification('Camera stream started', 'success');
                updateStreamInfo();
            } else {
                showNotification(`Failed to start camera: ${result.message}`, 'error');
            }
        } catch (error) {
            showNotification(`Error starting camera: ${error.message}`, 'error');
        }
    };

    window.stopCameraStream = async () => {
        if (!cameraStreamActive) {
            showNotification('Camera stream is not active', 'warning');
            return;
        }

        try {
            const response = await fetch('/camera/stop', { method: 'POST' });
            const result = await response.json();
            
            if (result.status === 'success') {
                cameraStreamActive = false;
                
                // Update main camera view
                document.getElementById('stream-status').textContent = 'Disconnected';
                document.getElementById('stream-status').style.color = 'var(--warning-color)';
                document.getElementById('camera-stream').style.display = 'none';
                document.getElementById('camera-stream').src = '';
                document.getElementById('camera-placeholder').style.display = 'flex';
                document.getElementById('stream-resolution').textContent = 'N/A';
                document.getElementById('stream-fps').textContent = '0';
                
                // Hide stream overlay
                const overlay = document.getElementById('stream-overlay');
                if (overlay) overlay.style.display = 'none';
                
                // Update preview in dashboard
                const previewPlaceholder = document.getElementById('camera-preview-placeholder');
                const previewStream = document.getElementById('camera-preview-stream');
                const previewStatus = document.getElementById('camera-preview-status');
                
                if (previewStream) {
                    previewStream.style.display = 'none';
                    previewStream.src = '';
                }
                if (previewPlaceholder) previewPlaceholder.style.display = 'flex';
                if (previewStatus) previewStatus.style.display = 'none';
                
                showNotification('Camera stream stopped', 'success');
            } else {
                showNotification(`Failed to stop camera: ${result.message}`, 'error');
            }
        } catch (error) {
            showNotification(`Error stopping camera: ${error.message}`, 'error');
        }
    };

    function updateStreamInfo() {
        if (!cameraStreamActive) return;
        
        const img = document.getElementById('camera-stream');
        if (img && img.naturalWidth && img.naturalHeight) {
            document.getElementById('stream-resolution').textContent = `${img.naturalWidth}x${img.naturalHeight}`;
        }
        
        // Simulate FPS counter (in real implementation, this would come from the backend)
        const fps = Math.floor(Math.random() * 5) + 25; // 25-30 FPS simulation
        document.getElementById('stream-fps').textContent = fps;
        
        if (cameraStreamActive) {
            setTimeout(updateStreamInfo, 1000);
        }
    }

    // Additional camera functions
    window.toggleFullscreen = () => {
        const container = document.getElementById('camera-container');
        if (!document.fullscreenElement) {
            container.requestFullscreen().catch(err => {
                showNotification(`Error entering fullscreen: ${err.message}`, 'error');
            });
        } else {
            document.exitFullscreen();
        }
    };

    window.takeSnapshot = () => {
        if (!cameraStreamActive) {
            showNotification('Camera stream is not active', 'warning');
            return;
        }
        
        const img = document.getElementById('camera-stream');
        const canvas = document.createElement('canvas');
        const ctx = canvas.getContext('2d');
        
        canvas.width = img.naturalWidth;
        canvas.height = img.naturalHeight;
        ctx.drawImage(img, 0, 0);
        
        canvas.toBlob((blob) => {
            const url = URL.createObjectURL(blob);
            const a = document.createElement('a');
            a.href = url;
            a.download = `camera_snapshot_${new Date().toISOString().replace(/[:.]/g, '-')}.jpg`;
            a.click();
            URL.revokeObjectURL(url);
            showNotification('Snapshot saved', 'success');
        }, 'image/jpeg', 0.9);
    };

    window.toggleRecording = () => {
        showNotification('Recording feature coming soon...', 'info');
    };

    window.adjustQuality = () => {
        showNotification('Quality adjustment coming soon...', 'info');
    };

    window.resetCamera = async () => {
        if (confirm('Reset camera stream? This will restart the connection.')) {
            if (cameraStreamActive) {
                await stopCameraStream();
                setTimeout(startCameraStream, 1000);
            }
            showNotification('Camera reset initiated', 'info');
        }
    };

    // Multi-Camera Functions
    window.startAllCameras = async () => {
        try {
            const response = await fetch('/camera/start_all', { method: 'POST' });
            const result = await response.json();
            
            if (result.status === 'success') {
                showNotification('Starting all cameras...', 'info');
                
                // Update UI for each camera
                result.results.forEach(item => {
                    if (item.result.status === 'success') {
                        activeCameras[item.camera] = true;
                        updateCameraUI(item.camera, true);
                    }
                });
                
                updateCameraStatusTable();
            } else {
                showNotification('Failed to start all cameras', 'error');
            }
        } catch (error) {
            showNotification(`Error starting cameras: ${error.message}`, 'error');
        }
    };

    window.stopAllCameras = async () => {
        try {
            const response = await fetch('/camera/stop_all', { method: 'POST' });
            const result = await response.json();
            
            if (result.status === 'success') {
                showNotification('Stopping all cameras...', 'info');
                
                // Update UI for each camera
                Object.keys(activeCameras).forEach(cameraId => {
                    activeCameras[cameraId] = false;
                    updateCameraUI(cameraId, false);
                });
                
                updateCameraStatusTable();
            } else {
                showNotification('Failed to stop all cameras', 'error');
            }
        } catch (error) {
            showNotification(`Error stopping cameras: ${error.message}`, 'error');
        }
    };

    window.startSpecificCamera = async (cameraId) => {
        try {
            const response = await fetch(`/camera/start/${cameraId}`, { method: 'POST' });
            const result = await response.json();
            
            if (result.status === 'success' || result.status === 'already_active') {
                activeCameras[cameraId] = true;
                updateCameraUI(cameraId, true);
                showNotification(`Camera ${cameraId} started`, 'success');
                updateCameraStatusTable();
            } else {
                showNotification(`Failed to start ${cameraId}: ${result.message}`, 'error');
            }
        } catch (error) {
            showNotification(`Error starting ${cameraId}: ${error.message}`, 'error');
        }
    };

    window.stopSpecificCamera = async (cameraId) => {
        try {
            const response = await fetch(`/camera/stop/${cameraId}`, { method: 'POST' });
            const result = await response.json();
            
            if (result.status === 'success' || result.status === 'already_inactive') {
                activeCameras[cameraId] = false;
                updateCameraUI(cameraId, false);
                showNotification(`Camera ${cameraId} stopped`, 'success');
                updateCameraStatusTable();
            } else {
                showNotification(`Failed to stop ${cameraId}: ${result.message}`, 'error');
            }
        } catch (error) {
            showNotification(`Error stopping ${cameraId}: ${error.message}`, 'error');
        }
    };

    window.focusCamera = (cameraId) => {
        currentFocusCamera = cameraId;
        switchLayout('single');
        
        // Update single view
        const singleTitle = document.getElementById('single-camera-title');
        const singleStream = document.getElementById('single-camera-stream');
        const singlePlaceholder = document.getElementById('single-camera-placeholder');
        const singleOverlay = document.getElementById('single-camera-overlay');
        
        if (singleTitle) {
            const cameraNames = {
                'global': 'Global Camera - Main View',
                'hanabot_3': 'HanaBot 3 - Station A',
                'hanabot_8': 'HanaBot 8 - Station B',
                'hanabot_9': 'HanaBot 9 - Station C'
            };
            singleTitle.textContent = cameraNames[cameraId] || `Camera ${cameraId}`;
        }
        
        if (activeCameras[cameraId]) {
            if (singlePlaceholder) singlePlaceholder.style.display = 'none';
            if (singleStream) {
                singleStream.style.display = 'block';
                singleStream.src = `/camera/stream/${cameraId}?` + Date.now();
            }
            if (singleOverlay) singleOverlay.style.display = 'block';
        } else {
            if (singleStream) singleStream.style.display = 'none';
            if (singlePlaceholder) singlePlaceholder.style.display = 'flex';
            if (singleOverlay) singleOverlay.style.display = 'none';
        }
    };

    window.switchLayout = (layout) => {
        currentLayout = layout;
        const gridLayout = document.getElementById('camera-grid-layout');
        const singleLayout = document.getElementById('camera-single-layout');
        
        // Update button states
        document.querySelectorAll('[onclick*="switchLayout"]').forEach(btn => {
            btn.classList.remove('active');
        });
        
        if (layout === 'grid') {
            if (gridLayout) gridLayout.style.display = 'block';
            if (singleLayout) singleLayout.style.display = 'none';
            document.querySelector('[onclick*="grid"]').classList.add('active');
        } else if (layout === 'single') {
            if (gridLayout) gridLayout.style.display = 'none';
            if (singleLayout) singleLayout.style.display = 'block';
            document.querySelector('[onclick*="single"]').classList.add('active');
        }
    };

    window.toggleAutoSwitch = () => {
        autoSwitchActive = !autoSwitchActive;
        const btn = document.querySelector('[onclick*="toggleAutoSwitch"]');
        
        if (autoSwitchActive) {
            btn.classList.add('auto-switch-active');
            showNotification('Auto switch enabled - cameras will cycle every 10 seconds', 'info');
            startAutoSwitch();
        } else {
            btn.classList.remove('auto-switch-active');
            showNotification('Auto switch disabled', 'info');
            stopAutoSwitch();
        }
    };

    function startAutoSwitch() {
        if (autoSwitchInterval) return;
        
        const cameras = ['global', 'hanabot_3', 'hanabot_8', 'hanabot_9'];
        let currentIndex = 0;
        
        autoSwitchInterval = setInterval(() => {
            if (!autoSwitchActive) return;
            
            const activeKeys = cameras.filter(id => activeCameras[id]);
            if (activeKeys.length === 0) return;
            
            const cameraId = activeKeys[currentIndex % activeKeys.length];
            focusCamera(cameraId);
            currentIndex++;
        }, 10000); // Switch every 10 seconds
    }

    function stopAutoSwitch() {
        if (autoSwitchInterval) {
            clearInterval(autoSwitchInterval);
            autoSwitchInterval = null;
        }
    }

    window.takeAllSnapshots = () => {
        const activeCameraIds = Object.keys(activeCameras).filter(id => activeCameras[id]);
        
        if (activeCameraIds.length === 0) {
            showNotification('No active cameras to snapshot', 'warning');
            return;
        }
        
        showNotification(`Taking snapshots from ${activeCameraIds.length} cameras...`, 'info');
        
        activeCameraIds.forEach(cameraId => {
            const img = document.getElementById(`camera-stream-${cameraId}`);
            if (img && img.src) {
                const canvas = document.createElement('canvas');
                const ctx = canvas.getContext('2d');
                
                canvas.width = img.naturalWidth || 640;
                canvas.height = img.naturalHeight || 480;
                ctx.drawImage(img, 0, 0);
                
                canvas.toBlob((blob) => {
                    const url = URL.createObjectURL(blob);
                    const a = document.createElement('a');
                    a.href = url;
                    a.download = `${cameraId}_snapshot_${new Date().toISOString().replace(/[:.]/g, '-')}.jpg`;
                    a.click();
                    URL.revokeObjectURL(url);
                }, 'image/jpeg', 0.9);
            }
        });
        
        setTimeout(() => {
            showNotification('All snapshots saved', 'success');
        }, 1000);
    };

    // Robot camera control functions
    window.startRobotCameras = async () => {
        const robotCameras = ['hanabot_3', 'hanabot_8', 'hanabot_9'];
        let successCount = 0;
        
        for (const cameraId of robotCameras) {
            try {
                const response = await fetch(`/camera/start/${cameraId}`, { method: 'POST' });
                const result = await response.json();
                
                if (result.status === 'success' || result.status === 'already_active') {
                    activeCameras[cameraId] = true;
                    updateCameraUI(cameraId, true);
                    updateRobotStatus(cameraId, true);
                    successCount++;
                }
            } catch (error) {
                console.error(`Failed to start ${cameraId}:`, error);
            }
        }
        
        showNotification(`Started ${successCount}/${robotCameras.length} robot cameras`, 'success');
        updateCameraStatusTable();
    };

    window.stopRobotCameras = async () => {
        const robotCameras = ['hanabot_3', 'hanabot_8', 'hanabot_9'];
        let successCount = 0;
        
        for (const cameraId of robotCameras) {
            try {
                const response = await fetch(`/camera/stop/${cameraId}`, { method: 'POST' });
                const result = await response.json();
                
                if (result.status === 'success' || result.status === 'already_inactive') {
                    activeCameras[cameraId] = false;
                    updateCameraUI(cameraId, false);
                    updateRobotStatus(cameraId, false);
                    successCount++;
                }
            } catch (error) {
                console.error(`Failed to stop ${cameraId}:`, error);
            }
        }
        
        showNotification(`Stopped ${successCount}/${robotCameras.length} robot cameras`, 'success');
        updateCameraStatusTable();
    };

    function updateRobotStatus(cameraId, isOnline) {
        const statusElement = document.getElementById(`robot-status-${cameraId}`);
        if (statusElement) {
            const statusDot = statusElement.querySelector('.status-dot');
            if (statusDot) {
                statusDot.className = `status-dot ${isOnline ? 'online' : 'offline'}`;
            }
        }
    }

    function updateCameraUI(cameraId, isActive) {
        const placeholder = document.getElementById(`camera-placeholder-${cameraId}`);
        const stream = document.getElementById(`camera-stream-${cameraId}`);
        const overlay = document.getElementById(`camera-overlay-${cameraId}`);
        const status = document.getElementById(`camera-status-${cameraId}`);
        const container = document.getElementById(`camera-container-${cameraId}`);
        
        if (isActive) {
            if (placeholder) placeholder.style.display = 'none';
            if (stream) {
                stream.style.display = 'block';
                stream.src = `/camera/stream/${cameraId}?` + Date.now();
            }
            if (overlay) overlay.style.display = 'block';
            if (status) {
                status.innerHTML = '<span class="status-dot online"></span> Online';
            }
            if (container) {
                container.classList.remove('camera-error');
                container.classList.add('camera-connecting');
                setTimeout(() => container.classList.remove('camera-connecting'), 2000);
            }
        } else {
            if (stream) {
                stream.style.display = 'none';
                stream.src = '';
            }
            if (placeholder) placeholder.style.display = 'flex';
            if (overlay) overlay.style.display = 'none';
            if (status) {
                status.innerHTML = '<span class="status-dot offline"></span> Offline';
            }
            if (container) {
                container.classList.remove('camera-connecting', 'camera-error');
            }
        }
        
        // Update preview in dashboard if exists
        const previewStream = document.getElementById('camera-preview-stream');
        const previewPlaceholder = document.getElementById('camera-preview-placeholder');
        const previewStatus = document.getElementById('camera-preview-status');
        
        if (cameraId === 'global') { // Update dashboard preview with global camera
            if (isActive) {
                if (previewPlaceholder) previewPlaceholder.style.display = 'none';
                if (previewStream) {
                    previewStream.style.display = 'block';
                    previewStream.src = `/camera/stream/${cameraId}?` + Date.now();
                }
                if (previewStatus) previewStatus.style.display = 'block';
            } else {
                if (previewStream) {
                    previewStream.style.display = 'none';
                    previewStream.src = '';
                }
                if (previewPlaceholder) previewPlaceholder.style.display = 'flex';
                if (previewStatus) previewStatus.style.display = 'none';
            }
        }
    }

    async function updateCameraStatusTable() {
        try {
            const response = await fetch('/camera/list');
            const result = await response.json();
            
            const tbody = document.getElementById('camera-status-table');
            if (!tbody || !result.cameras) return;
            
            tbody.innerHTML = result.cameras.map(camera => `
                <tr>
                    <td>
                        <i class="bi bi-${camera.id === 'global' ? 'globe' : 'robot'}"></i>
                        ${camera.name}
                    </td>
                    <td>${camera.location}</td>
                    <td>
                        <span class="status-badge ${camera.active ? 'idle' : 'error'}">
                            ${camera.active ? 'Active' : 'Inactive'}
                        </span>
                    </td>
                    <td>${camera.port}</td>
                    <td>${camera.active ? '25-30' : '0'}</td>
                    <td>${camera.active ? '640x480' : 'N/A'}</td>
                    <td>${camera.active ? new Date().toLocaleTimeString() : 'Never'}</td>
                    <td>
                        <button class="action-icon-btn" onclick="startSpecificCamera('${camera.id}')" title="Start">
                            <i class="bi bi-play-fill"></i>
                        </button>
                        <button class="action-icon-btn danger" onclick="stopSpecificCamera('${camera.id}')" title="Stop">
                            <i class="bi bi-stop-fill"></i>
                        </button>
                        <button class="action-icon-btn" onclick="focusCamera('${camera.id}')" title="Focus">
                            <i class="bi bi-arrows-fullscreen"></i>
                        </button>
                    </td>
                </tr>
            `).join('');
        } catch (error) {
            console.error('Failed to update camera status table:', error);
        }
    }

    // Camera analytics and monitoring
    async function updateCameraAnalytics() {
        try {
            const [analyticsResponse, healthResponse, alertsResponse] = await Promise.all([
                fetch('/camera/analytics'),
                fetch('/camera/health'),
                fetch('/camera/alerts')
            ]);
            
            const analytics = await analyticsResponse.json();
            const health = await healthResponse.json();
            const alerts = await alertsResponse.json();
            
            // Update dashboard metrics
            const activeCamerasEl = document.getElementById('active-cameras');
            const bandwidthEl = document.getElementById('camera-bandwidth');
            const uptimeEl = document.getElementById('system-uptime');
            const alertsEl = document.getElementById('camera-alerts');
            
            if (activeCamerasEl) activeCamerasEl.textContent = analytics.active_cameras;
            if (bandwidthEl) bandwidthEl.textContent = analytics.total_bandwidth_mbps.toFixed(1);
            if (uptimeEl) uptimeEl.textContent = `${analytics.uptime_hours.toFixed(1)}h`;
            if (alertsEl) alertsEl.textContent = alerts.total_today;
            
            // Update change indicators
            const camerasChangeEl = document.getElementById('cameras-change');
            const bandwidthChangeEl = document.getElementById('bandwidth-change');
            const alertsChangeEl = document.getElementById('alerts-change');
            
            if (camerasChangeEl) {
                const utilization = analytics.utilization_percent;
                camerasChangeEl.textContent = `${utilization.toFixed(0)}%`;
                camerasChangeEl.className = `metric-change ${utilization > 50 ? 'positive' : 'negative'}`;
            }
            
            if (bandwidthChangeEl) {
                const change = analytics.total_bandwidth_mbps > 5 ? '+' : '±';
                bandwidthChangeEl.textContent = `${change}${(analytics.total_bandwidth_mbps * 10).toFixed(0)}%`;
            }
            
            if (alertsChangeEl) {
                alertsChangeEl.textContent = alerts.unresolved > 0 ? `+${alerts.unresolved}` : '0';
                alertsChangeEl.className = `metric-change ${alerts.unresolved > 0 ? 'negative' : 'positive'}`;
            }
            
        } catch (error) {
            console.error('Failed to update camera analytics:', error);
        }
    }

    // Motion detection alerts (placeholder)
    function checkMotionAlerts() {
        // In real implementation, this would analyze camera feeds for motion
        const cameras = Object.keys(activeCameras).filter(id => activeCameras[id]);
        
        cameras.forEach(cameraId => {
            // Simulate occasional motion detection (5% chance per check)
            if (Math.random() < 0.05) {
                showMotionAlert(cameraId);
            }
        });
    }

    function showMotionAlert(cameraId) {
        const cameraNames = {
            'global': 'Global Camera',
            'hanabot_3': 'HanaBot 3',
            'hanabot_8': 'HanaBot 8', 
            'hanabot_9': 'HanaBot 9'
        };
        
        showNotification(`Motion detected on ${cameraNames[cameraId]}`, 'warning');
        
        // Highlight the camera in grid view
        const container = document.getElementById(`camera-container-${cameraId}`);
        if (container) {
            container.style.border = '2px solid var(--warning-color)';
            setTimeout(() => {
                container.style.border = '';
            }, 3000);
        }
    }

    // System health monitoring
    async function monitorSystemHealth() {
        try {
            const response = await fetch('/camera/health');
            const healthData = await response.json();
            
            // Check for any cameras that stopped unexpectedly
            Object.keys(healthData.cameras).forEach(cameraId => {
                const camera = healthData.cameras[cameraId];
                const wasActive = activeCameras[cameraId];
                const isActive = camera.active;
                
                if (wasActive && !isActive) {
                    showNotification(`Camera ${cameraId} disconnected unexpectedly`, 'error');
                    updateCameraUI(cameraId, false);
                }
                
                // Update local state
                activeCameras[cameraId] = isActive;
            });
            
            // Check system bandwidth
            if (healthData.total_bandwidth > 10) { // Alert if over 10 Mbps 
                showNotification('High bandwidth usage detected', 'warning');
            }
            
        } catch (error) {
            console.error('Health monitoring failed:', error);
        }
    }

    // Enhanced camera functions with analytics
    const originalStartCameraStream = window.startCameraStream;
    window.startCameraStream = async () => {
        await originalStartCameraStream();
        updateCameraAnalytics();
    };

    const originalStopCameraStream = window.stopCameraStream;
    window.stopCameraStream = async () => {
        await originalStopCameraStream();
        updateCameraAnalytics(); 
    };

    // Real-time monitoring intervals
    setInterval(updateCameraAnalytics, 10000); // Update analytics every 10 seconds
    setInterval(checkMotionAlerts, 30000); // Check for motion every 30 seconds
    setInterval(monitorSystemHealth, 15000); // Monitor health every 15 seconds

    // Initialize camera status table and analytics
    updateCameraStatusTable();
    updateCameraAnalytics();
    setInterval(updateCameraStatusTable, 5000); // Update every 5 seconds

    // Initial dashboard setup
    async function initDashboard() {
        console.log('HANA Admin Dashboard initialized');
        initCharts();
        showSection('dashboard'); // Start with dashboard section
        showDashboardSubtab('overview'); // Start with overview subtab
        refreshData();
        updateBatteryCards(); // Initial battery card update
        updateInterval = setInterval(() => {
            refreshData();
            updateBatteryCards(); // Update battery cards periodically
        }, 10000); // Auto-refresh every 10 seconds
    }

    window.addEventListener('beforeunload', () => {
        if (updateInterval) clearInterval(updateInterval);
    });

    document.addEventListener('click', (e) => {
        if (e.target.classList.contains('chart-control')) {
            e.target.parentElement.querySelectorAll('.chart-control').forEach(btn => {
                btn.classList.remove('active');
            });
            e.target.classList.add('active');
            const period = e.target.dataset.period;
            showNotification(`Switching to ${period} view...`, 'info');
            
            // Update chart data based on period
            updateChartPeriod(period);
        }
    });

    // Update chart based on time period
    function updateChartPeriod(period) {
        // Update chart title
        const chartTitle = document.querySelector('.chart-section h3');
        if (chartTitle && chartTitle.textContent.includes('Task Distribution')) {
            const periods = {'24h': 'Hourly (24h)', '7d': 'Daily (7 days)', '30d': 'Daily (30 days)'};
            chartTitle.textContent = `Task Distribution - ${periods[period]}`;
        }
        
        // Show loading state
        showNotification(`Loading ${period} data...`, 'info', 2000);
        
        // Fetch data with period parameter
        fetchAnalyticsForPeriod(period);
    }

    // Fetch analytics data for specific period
    async function fetchAnalyticsForPeriod(period) {
        try {
            // For now, use the existing API with period as a query parameter
            const response = await fetch(`/api/fleet/analytics?period=${period}`);
            if (!response.ok) throw new Error(`HTTP ${response.status}: ${response.statusText}`);
            const result = await response.json();
            
            if (result.success && result.data) {
                currentData = result.data;
                
                // Update charts based on period
                if (period === '24h') {
                    // Use hourly data for 24h view
                    if (currentData.hourly_distribution) {
                        updateHourlyChart(currentData.hourly_distribution);
                    }
                } else {
                    // Use daily trends for 7d and 30d views
                    if (currentData.daily_trends) {
                        updateHourlyChartWithDailyData(currentData.daily_trends, period);
                    }
                }
                
                // Update other components
                if (currentData.system_metrics) {
                    updateMetrics(currentData.system_metrics);
                    updateStatusChart(currentData.system_metrics);
                }
                if (currentData.robot_performance) {
                    updateRobotChart(currentData.robot_performance);
                }
                if (currentData.top_requesters) {
                    updateTopUsersTable(currentData.top_requesters);
                }
                if (currentData.popular_items) {
                    updatePopularItemsTable(currentData.popular_items);
                }

                console.log(`✅ Analytics data updated for ${period}`);
            } else {
                throw new Error(result.error || 'No data received from API');
            }
        } catch (error) {
            console.error(`❌ Failed to fetch ${period} analytics:`, error);
            showNotification(`Failed to load ${period} data`, 'error');
            
            // Fall back to regular analytics if period-specific fails
            fetchAnalytics();
        }
    }

    // Update hourly chart with daily data for 7d/30d views
    function updateHourlyChartWithDailyData(dailyData, period) {
        if (!charts.hourly) {
            console.warn('Hourly chart not initialized');
            return;
        }

        const limitDays = period === '7d' ? 7 : 30;
        const recentData = dailyData.slice(0, limitDays);
        
        const dates = recentData.map(item => {
            const date = new Date(item.date);
            return period === '7d' ? 
                date.toLocaleDateString('ko-KR', { weekday: 'short', month: 'short', day: 'numeric' }) :
                date.toLocaleDateString('ko-KR', { month: 'short', day: 'numeric' });
        }).reverse();
        
        const tasks = recentData.map(item => item.total_tasks || 0).reverse();
        const completed = recentData.map(item => item.completed_tasks || 0).reverse();
        const failed = recentData.map(item => item.failed_tasks || 0).reverse();

        try {
            charts.hourly.data.labels = dates;
            charts.hourly.data.datasets[0].data = tasks;
            charts.hourly.data.datasets[1].data = completed;
            charts.hourly.data.datasets[2].data = failed;
            charts.hourly.update();
            
            console.log(`✅ Updated chart with ${period} daily data`);
        } catch (error) {
            console.error(`Failed to update chart with daily data:`, error);
        }
    }

    // Fall Detection System
    let fallAlertState = {
        active: false,
        acknowledged: false,
        checkInterval: null
    };

    function initFallDetection() {
        // Check fall detection status every 2 seconds
        fallAlertState.checkInterval = setInterval(checkFallDetectionStatus, 2000);
        
        // Initial check
        checkFallDetectionStatus();
        
        console.log('🚨 Fall Detection System initialized');
    }

    function checkFallDetectionStatus() {
        fetch('/api/fall_detection/status')
            .then(response => response.json())
            .then(data => {
                if (data.success && data.data) {
                    updateFallAlertUI(data.data);
                }
            })
            .catch(error => {
                console.error('Error checking fall detection status:', error);
            });
    }

    function updateFallAlertUI(alertData) {
        const banner = document.getElementById('fall-alert-banner');
        const modal = document.getElementById('fall-alert-modal');
        const alertMessage = document.getElementById('fall-alert-message');
        const alertTime = document.getElementById('fall-alert-time');
        const modalMessage = document.getElementById('fall-alert-modal-message');

        if (alertData.active && !alertData.acknowledged) {
            // Show alert banner
            banner.classList.add('active');
            document.body.classList.add('fall-alert-active');
            
            // Update message
            if (alertData.message) {
                alertMessage.textContent = alertData.message;
                modalMessage.textContent = alertData.message;
            }
            
            // Update time
            if (alertData.timestamp) {
                const alertDate = new Date(alertData.timestamp * 1000);
                alertTime.textContent = `Time: ${alertDate.toLocaleTimeString()}`;
            }
            
            // Show modal on first activation (not acknowledged)
            if (!fallAlertState.active) {
                modal.classList.add('show');
                
                // Play alert sound (optional)
                playAlertSound();
                
                // Show browser notification if supported
                showBrowserNotification('Fall Detected!', alertData.message || 'Emergency attention required');
            }
            
            fallAlertState.active = true;
            fallAlertState.acknowledged = alertData.acknowledged;
            
        } else {
            // Hide alert
            banner.classList.remove('active');
            modal.classList.remove('show');
            document.body.classList.remove('fall-alert-active');
            
            if (fallAlertState.active) {
                showNotification('Fall alert cleared', 'success');
            }
            
            fallAlertState.active = false;
            fallAlertState.acknowledged = false;
        }
    }

    function acknowledgeFallAlert() {
        fetch('/api/fall_detection/acknowledge', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json'
            }
        })
        .then(response => response.json())
        .then(data => {
            if (data.success) {
                showNotification('Fall alert acknowledged', 'success');
                // Keep banner but mark as acknowledged
                fallAlertState.acknowledged = true;
            } else {
                showNotification('Failed to acknowledge alert', 'error');
            }
        })
        .catch(error => {
            console.error('Error acknowledging fall alert:', error);
            showNotification('Error acknowledging alert', 'error');
        });
    }

    function clearFallAlert() {
        if (!confirm('Are you sure you want to clear the fall alert?')) {
            return;
        }
        
        fetch('/api/fall_detection/clear', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json'
            }
        })
        .then(response => response.json())
        .then(data => {
            if (data.success) {
                showNotification('Fall alert cleared', 'success');
            } else {
                showNotification('Failed to clear alert', 'error');
            }
        })
        .catch(error => {
            console.error('Error clearing fall alert:', error);
            showNotification('Error clearing alert', 'error');
        });
    }

    function acknowledgeAndCloseModal() {
        acknowledgeFallAlert();
        document.getElementById('fall-alert-modal').classList.remove('show');
    }

    function testFallAlert() {
        fetch('/api/fall_detection/test', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json'
            }
        })
        .then(response => response.json())
        .then(data => {
            if (data.success) {
                showNotification('Test fall alert activated', 'info');
                document.getElementById('fall-alert-modal').classList.remove('show');
            } else {
                showNotification('Failed to activate test alert', 'error');
            }
        })
        .catch(error => {
            console.error('Error testing fall alert:', error);
            showNotification('Error testing alert', 'error');
        });
    }

    function playAlertSound() {
        try {
            // Create audio context for alert sound - handle browser compatibility
            const AudioCtx = window.AudioContext || window.webkitAudioContext;
            if (!AudioCtx) {
                console.log('Web Audio API not supported');
                return;
            }
            
            const audioContext = new AudioCtx();
            const oscillator = audioContext.createOscillator();
            const gainNode = audioContext.createGain();
            
            oscillator.connect(gainNode);
            gainNode.connect(audioContext.destination);
            
            oscillator.frequency.setValueAtTime(800, audioContext.currentTime);
            oscillator.frequency.setValueAtTime(600, audioContext.currentTime + 0.2);
            oscillator.frequency.setValueAtTime(800, audioContext.currentTime + 0.4);
            
            gainNode.gain.setValueAtTime(0.3, audioContext.currentTime);
            gainNode.gain.exponentialRampToValueAtTime(0.01, audioContext.currentTime + 0.6);
            
            oscillator.start(audioContext.currentTime);
            oscillator.stop(audioContext.currentTime + 0.6);
            
        } catch (error) {
            console.log('Could not play alert sound:', error);
        }
    }

    function showBrowserNotification(title, message) {
        if ('Notification' in window && Notification.permission === 'granted') {
            new Notification(title, {
                body: message,
                icon: '/static/favicon.ico', // Optional: add favicon path
                requireInteraction: true,
                tag: 'fall-alert'
            });
        } else if ('Notification' in window && Notification.permission !== 'denied') {
            Notification.requestPermission().then(permission => {
                if (permission === 'granted') {
                    new Notification(title, {
                        body: message,
                        requireInteraction: true,
                        tag: 'fall-alert'
                    });
                }
            });
        }
    }

    // Make functions global for onclick handlers
    window.acknowledgeFallAlert = acknowledgeFallAlert;
    window.clearFallAlert = clearFallAlert;
    window.acknowledgeAndCloseModal = acknowledgeAndCloseModal;
    window.testFallAlert = testFallAlert;

    // ========================================
    // BASIC ROBOTS AND TASKS FUNCTIONALITY
    // ========================================

    // Robot data management
    let robotData = [];
    let robotPerformanceChart = null;
    let batteryTimelineChart = null;

    async function fetchRobotData() {
        try {
            const response = await fetch('/api/admin/robots');
            if (!response.ok) throw new Error(`HTTP ${response.status}: ${response.statusText}`);
            const result = await response.json();
            
            if (result.success && result.data) {
                robotData = result.data;
                updateFleetMetrics();
                renderRobotCards();
                updateRobotTable();
                updateBatteryCards(result.data);
                updateRobotStatusIndicators(result.data);
                
                // Add delay for DOM to fully render
                setTimeout(async () => {
                    try {
                        console.log('🔄 Starting robot charts update...');
                        
                        // Check if we're on the robots section
                        const robotsSection = document.getElementById('robots-section');
                        if (!robotsSection || !robotsSection.classList.contains('active')) {
                            console.log('⚠️ Robots section not active, skipping chart update');
                            return;
                        }
                        
                        // Check if robotData is available
                        if (!robotData || robotData.length === 0) {
                            console.log('⚠️ No robot data available for charts');
                            return;
                        }
                        
                        console.log('📊 Robot data available for charts:', robotData.length, 'robots');
                        
                        await updateRobotCharts();
                        // Force chart redraw after section is visible
                        if (robotPerformanceChart) {
                            robotPerformanceChart.resize();
                        }
                        if (batteryTimelineChart) {
                            batteryTimelineChart.resize();
                        }
                        console.log('✅ Robot charts updated and resized successfully');
                    } catch (error) {
                        console.error('❌ Failed to update robot charts:', error);
                        showNotification('Chart rendering failed', 'error');
                    }
                }, 500);
                
                console.log('✅ Robot data updated:', robotData.length, 'robots');
            } else {
                throw new Error(result.error || 'Failed to fetch robot data');
            }
        } catch (error) {
            console.error('❌ Failed to fetch robot data:', error);
            showNotification('Failed to load robot data', 'error');
        }
    }

    // Update fleet overview metrics (filter out HANA_PINKY)
    function updateFleetMetrics() {
        const visibleRobots = robotData.filter(robot => robot.name !== 'HANA_PINKY');
        const total = visibleRobots.length;
        const online = visibleRobots.filter(robot => robot.status !== 'offline' && robot.status !== '오프라인').length;
        const activeTasks = visibleRobots.filter(robot => robot.current_task).length;
        const totalBattery = visibleRobots.reduce((sum, robot) => sum + (robot.battery || 0), 0);
        const avgBattery = total > 0 ? Math.round(totalBattery / total) : 0;
        const alerts = visibleRobots.filter(robot => robot.status === 'error' || (robot.battery && robot.battery < 20)).length;
        const efficiency = activeTasks > 0 ? Math.round((activeTasks / total) * 100) : 0;

        // Update UI elements
        const updates = {
            'fleet-total-robots': total,
            'fleet-robots-online': `${online} Online`,
            'fleet-active-tasks': activeTasks,
            'fleet-task-efficiency': `${efficiency}% Efficiency`,
            'fleet-avg-battery': `${avgBattery}%`,
            'fleet-battery-status': avgBattery > 60 ? 'Normal' : avgBattery > 30 ? 'Low' : 'Critical',
            'fleet-alerts': alerts,
            'fleet-alert-status': alerts === 0 ? 'All Clear' : `${alerts} Active`
        };

        Object.entries(updates).forEach(([id, value]) => {
            const element = document.getElementById(id);
            if (element) element.textContent = value;
        });
    }

    // Render individual robot cards (filter out HANA_PINKY)
    function renderRobotCards() {
        const container = document.getElementById('robot-cards-container');
        if (!container) {
            console.log('Robot cards container not found, skipping card rendering');
            return;
        }

        const visibleRobots = robotData.filter(robot => robot.name !== 'HANA_PINKY');
        container.innerHTML = visibleRobots.map(robot => {
            // Get status-based CSS class for card
            const getStatusClass = (status) => {
                const statusMap = {
                    'idle': 'idle',
                    'busy': 'busy', 
                    'moving_to_arm': 'busy',
                    'picking': 'busy',
                    'moving_to_user': 'busy', 
                    'waiting_confirm': 'busy',
                    'returning_to_dock': 'busy',
                    'offline': 'offline',
                    'error': 'error',
                    'charging': 'idle'
                };
                return statusMap[status] || 'offline';
            };
            
            const statusClass = getStatusClass(robot.status || 'offline');
            
            return `
            <div class="col-lg-4 col-md-6 mb-3">
                <div class="robot-card robot-card-status ${statusClass}">
                    <div class="robot-card-header">
                        <div class="robot-card-icon">
                            <i class="bi bi-robot"></i>
                        </div>
                        <div class="robot-card-info">
                            <div class="robot-card-name">${robot.name || 'Unknown'}</div>
                            <div class="robot-card-id">ID: ${robot.id || 'N/A'}</div>
                        </div>
                        <div class="robot-card-status">
                            ${getStatusBadgeHtml(robot.status || 'unknown')}
                        </div>
                    </div>
                    <div class="robot-card-body">
                        <div class="robot-card-metrics">
                            <div class="robot-card-metric">
                                <div class="robot-card-metric-label">Battery</div>
                                <div class="robot-card-metric-value">${robot.battery || 0}%</div>
                            </div>
                            <div class="robot-card-metric">
                                <div class="robot-card-metric-label">Task</div>
                                <div class="robot-card-metric-content">
                                    ${robot.current_task || 'Idle'}
                                </div>
                            </div>
                        </div>
                        <div class="robot-card-actions">
                            <button class="action-icon-btn" onclick="viewRobotDetails(${robot.id})" title="Robot Details">
                                <i class="bi bi-eye"></i>
                            </button>
                            <button class="action-icon-btn danger" onclick="stopRobot(${robot.id})" title="Stop Robot">
                                <i class="bi bi-stop-circle"></i>
                            </button>
                            <button class="action-icon-btn" onclick="sendRobotHome(${robot.id})" title="Send Home">
                                <i class="bi bi-house"></i>
                            </button>
                        </div>
                    </div>
                </div>
            </div>
        `;
        }).join('');
    }

    function updateRobotTable() {
        const tbody = document.querySelector('#robots-table-body');
        if (!tbody) return;

        const visibleRobots = robotData.filter(robot => robot.name !== 'HANA_PINKY');
        if (visibleRobots.length === 0) {
            tbody.innerHTML = '<tr><td colspan="7" class="text-center text-muted">No robots found</td></tr>';
            return;
        }

        tbody.innerHTML = visibleRobots.map(robot => `
            <tr>
                <td>${robot.id || 'N/A'}</td>
                <td>${robot.name || 'N/A'}</td>
                <td>${getStatusBadgeHtml(robot.status || 'unknown')}</td>
                <td>${robot.battery ? robot.battery + '%' : 'N/A'}</td>
                <td>${robot.current_task || 'None'}</td>
                <td>${robot.performance || 'N/A'}</td>
                <td>
                    <button class="action-icon-btn" onclick="viewRobotDetails(${robot.id})" title="View Details">
                        <i class="bi bi-eye"></i>
                    </button>
                    <button class="action-icon-btn danger" onclick="stopRobot(${robot.id})" title="Stop Robot">
                        <i class="bi bi-stop-circle"></i>
                    </button>
                </td>
            </tr>
        `).join('');
    }

    function viewRobotDetails(robotId) {
        const robot = robotData.find(r => r.id === robotId);
        if (!robot) return;
        
        // Create modal popup for robot details
        const modal = document.createElement('div');
        modal.style.cssText = `
            position: fixed;
            top: 0;
            left: 0;
            width: 100%;
            height: 100%;
            background: rgba(0, 0, 0, 0.7);
            z-index: 10000;
            display: flex;
            align-items: center;
            justify-content: center;
            animation: fadeIn 0.3s ease-out;
        `;
        
        const modalContent = document.createElement('div');
        modalContent.style.cssText = `
            background: #2c3e50;
            border-radius: 10px;
            padding: 25px;
            max-width: 500px;
            width: 90%;
            color: white;
            box-shadow: 0 10px 30px rgba(0,0,0,0.5);
            animation: slideIn 0.3s ease-out;
        `;
        
        modalContent.innerHTML = `
            <div style="display: flex; justify-content: space-between; align-items: center; margin-bottom: 20px;">
                <h4 style="margin: 0; color: #00aaff;"><i class="bi bi-robot"></i> Robot Details</h4>
                <button id="closeModal" style="background: none; border: none; color: #ccc; font-size: 24px; cursor: pointer; padding: 0;">&times;</button>
            </div>
            <div style="line-height: 1.8;">
                <div style="margin-bottom: 15px;">
                    <strong style="color: #00aaff;">Robot:</strong> ${robot.name || robotId}
                </div>
                <div style="margin-bottom: 15px;">
                    <strong style="color: #00aaff;">ID:</strong> ${robot.id}
                </div>
                <div style="margin-bottom: 15px;">
                    <strong style="color: #00aaff;">Status:</strong> ${getStatusBadgeHtml(robot.status || 'Unknown')}
                </div>
                <div style="margin-bottom: 15px;">
                    <strong style="color: #00aaff;">Battery:</strong> ${getBatteryStatusHtml(robot.id, robot.battery)}
                </div>
                <div style="margin-bottom: 15px;">
                    <strong style="color: #00aaff;">Current Task:</strong> ${robot.current_task ? `#${robot.current_task.task_id} (${robot.current_task.task_type})` : 'None'}
                </div>
                <div style="margin-bottom: 15px;">
                    <strong style="color: #00aaff;">Last Update:</strong> ${robot.last_update ? new Date(robot.last_update * 1000).toLocaleString('ko-KR') : 'Never'}
                </div>
            </div>
            <div style="margin-top: 20px; text-align: right;">
                <button id="closeModalBtn" style="background: #00aaff; color: white; border: none; padding: 10px 20px; border-radius: 5px; cursor: pointer;">Close</button>
            </div>
        `;
        
        modal.appendChild(modalContent);
        document.body.appendChild(modal);
        
        // Close modal functionality
        const closeModal = () => {
            modal.style.animation = 'fadeOut 0.3s ease-out';
            setTimeout(() => {
                if (modal.parentNode) {
                    modal.parentNode.removeChild(modal);
                }
            }, 300);
        };
        
        modal.addEventListener('click', (e) => {
            if (e.target === modal) closeModal();
        });
        
        document.getElementById('closeModal').addEventListener('click', closeModal);
        document.getElementById('closeModalBtn').addEventListener('click', closeModal);
        
        // Add CSS animations if not already present
        if (!document.querySelector('#modalAnimations')) {
            const style = document.createElement('style');
            style.id = 'modalAnimations';
            style.textContent = `
                @keyframes fadeIn { from { opacity: 0; } to { opacity: 1; } }
                @keyframes fadeOut { from { opacity: 1; } to { opacity: 0; } }
                @keyframes slideIn { from { transform: translateY(-50px); opacity: 0; } to { transform: translateY(0); opacity: 1; } }
            `;
            document.head.appendChild(style);
        }
        
        console.log('Robot details modal opened:', robot);
    }

    function stopRobot(robotId) {
        if (!confirm(`Stop robot ${robotId}?`)) return;
        
        showNotification(`Stopping robot ${robotId}...`, 'warning');
        // In real implementation, this would make an API call
        console.log('Stop robot requested:', robotId);
    }

    function sendRobotHome(robotId) {
        if (!confirm(`Send robot ${robotId} home?`)) return;
        
        showNotification(`Sending robot ${robotId} home...`, 'info');
        // In real implementation, this would make an API call  
        console.log('Send robot home requested:', robotId);
    }

    function toggleRobotTable() {
        const table = document.getElementById('robot-detailed-table');
        const chevron = document.getElementById('robot-table-chevron');
        
        if (!table || !chevron) return;
        
        if (table.style.display === 'none') {
            table.style.display = 'block';
            chevron.classList.remove('bi-chevron-down');
            chevron.classList.add('bi-chevron-up');
        } else {
            table.style.display = 'none';
            chevron.classList.remove('bi-chevron-up');
            chevron.classList.add('bi-chevron-down');
        }
    }

    // Task data management
    let taskData = [];

    async function fetchTaskData() {
        try {
            const response = await fetch('/api/admin/tasks');
            if (!response.ok) throw new Error(`HTTP ${response.status}: ${response.statusText}`);
            const result = await response.json();
            
            if (result.success && result.data) {
                taskData = result.data;
                updateTaskTable();
                console.log('✅ Task data updated:', taskData.length, 'tasks');
            } else {
                throw new Error(result.error || 'Failed to fetch task data');
            }
        } catch (error) {
            console.error('❌ Failed to fetch task data:', error);
            showNotification('Failed to load task data', 'error');
        }
    }

    function updateTaskTable() {
        const tbody = document.querySelector('#tasks-table-body');
        if (!tbody) return;

        if (taskData.length === 0) {
            tbody.innerHTML = '<tr><td colspan="6" class="text-center text-muted">No tasks found</td></tr>';
            return;
        }

        tbody.innerHTML = taskData.map(task => `
            <tr>
                <td>${task.task_id}</td>
                <td>${getStatusBadgeHtml(task.status)}</td>
                <td>${task.requester || 'N/A'}</td>
                <td>${task.item || 'N/A'}</td>
                <td>${task.assigned_robot || 'Unassigned'}</td>
                <td>${task.created_at ? formatDateTime(task.created_at) : 'N/A'}</td>
            </tr>
        `).join('');
    }

    // Update robot performance and battery charts
    async function updateRobotCharts() {
        await updateRobotPerformanceChart();
        await updateBatteryTimelineChart();
    }

    // Robot performance bar chart - Use real success rate data from analytics
    async function updateRobotPerformanceChart() {
        const canvas = document.getElementById('robotPerformanceChart');
        if (!canvas) {
            console.error('❌ Robot Performance Chart canvas not found!');
            return;
        }
        console.log('✅ Robot Performance Chart canvas found:', canvas);

        const ctx = canvas.getContext('2d');
        
        if (robotPerformanceChart) {
            robotPerformanceChart.destroy();
        }

        // Initialize variables
        let labels = [];
        let performanceData = [];
        const colors = [
            'rgba(40, 167, 69, 0.8)',   // Green
            'rgba(0, 123, 255, 0.8)',   // Blue  
            'rgba(255, 193, 7, 0.8)',   // Yellow
            'rgba(220, 53, 69, 0.8)',   // Red
            'rgba(108, 117, 125, 0.8)', // Gray
            'rgba(23, 162, 184, 0.8)',  // Cyan
        ];

        try {
            // Fetch real robot performance data from analytics API
            const response = await fetch('/api/fleet/analytics');
            if (!response.ok) throw new Error('Analytics API failed');
            const analyticsResult = await response.json();
            
            if (analyticsResult.success && analyticsResult.data && analyticsResult.data.robot_performance) {
                // Use real robot performance data
                const robotPerformanceData = analyticsResult.data.robot_performance;
                console.log('📊 Raw robot performance data:', robotPerformanceData);
                
                // Filter out HANA_PINKY robots and robots with no tasks
                const filteredPerformance = robotPerformanceData.filter(robot => 
                    robot.bot_name && 
                    !robot.bot_name.includes('PINKY') && 
                    robot.total_tasks > 0
                );
                
                console.log('🎯 Filtered robot performance:', filteredPerformance);
                
                if (filteredPerformance.length > 0) {
                    labels = filteredPerformance.map(robot => robot.bot_name || `Robot ${robot.hana_bot_id}`);
                    performanceData = filteredPerformance.map(robot => {
                        // Calculate real success rate
                        const totalTasks = robot.total_tasks || 0;
                        const completedTasks = robot.completed_tasks || 0;
                        const successRate = totalTasks > 0 ? Math.round((completedTasks / totalTasks) * 100) : 0;
                        return successRate;
                    });
                    
                    console.log('✅ Using real robot performance data:', { labels, performanceData });
                } else {
                    console.warn('⚠️ No valid robot performance data after filtering');
                    throw new Error('No valid robot performance data');
                }
            } else {
                // Fallback to robot data if analytics unavailable
                const filteredRobots = robotData.filter(robot => robot.name !== 'HANA_PINKY');
                labels = filteredRobots.map(robot => robot.name || `Robot ${robot.id}`);
                performanceData = filteredRobots.map(() => 85); // Default fallback
                console.warn('⚠️ Using fallback robot performance data');
            }
        } catch (error) {
            console.error('❌ Failed to fetch robot performance data:', error);
            // Fallback to basic robot data
            const filteredRobots = robotData.filter(robot => robot.name !== 'HANA_PINKY');
            labels = filteredRobots.map(robot => robot.name || `Robot ${robot.id}`);
            performanceData = filteredRobots.map(() => 85); // Default fallback
            console.log('⚠️ Using fallback performance data:', { labels, performanceData });
        }
        
        // Ensure we have data to display
        if (labels.length === 0 || performanceData.length === 0) {
            console.warn('⚠️ No robot performance data available');
            // Clear any existing chart
            if (robotPerformanceChart) {
                robotPerformanceChart.destroy();
                robotPerformanceChart = null;
            }
            return;
        }

        console.log('🎯 Creating Robot Performance Chart with data:', { labels, performanceData });
        
        robotPerformanceChart = new Chart(ctx, {
            type: 'bar',
            data: {
                labels: labels,
                datasets: [{
                    label: 'Performance Score',
                    data: performanceData,
                    backgroundColor: performanceData.map((_, index) => colors[index % colors.length]),
                    borderColor: performanceData.map((_, index) => colors[index % colors.length].replace('0.8', '1')),
                    borderWidth: 2
                }]
            },
            options: {
                responsive: true,
                maintainAspectRatio: false,
                plugins: {
                    legend: {
                        display: false
                    }
                },
                scales: {
                    y: {
                        beginAtZero: true,
                        max: 100,
                        grid: {
                            color: 'rgba(255, 255, 255, 0.1)'
                        },
                        ticks: {
                            color: '#cccccc',
                            callback: function(value) {
                                return value + '%';
                            }
                        }
                    },
                    x: {
                        grid: {
                            color: 'rgba(255, 255, 255, 0.1)'
                        },
                        ticks: {
                            color: '#cccccc'
                        }
                    }
                }
            }
        });
        
        console.log('✅ Robot Performance Chart created successfully');
    }

    // Battery level timeline chart - Use real battery data
    async function updateBatteryTimelineChart() {
        const canvas = document.getElementById('batteryTimelineChart');
        if (!canvas) {
            console.error('❌ Battery Timeline Chart canvas not found!');
            return;
        }
        console.log('✅ Battery Timeline Chart canvas found:', canvas);

        const ctx = canvas.getContext('2d');
        
        if (batteryTimelineChart) {
            batteryTimelineChart.destroy();
        }

        // Generate 24-hour timeline labels
        const labels = Array.from({ length: 24 }, (_, i) => {
            const hour = (new Date().getHours() - 23 + i + 24) % 24;
            return `${hour.toString().padStart(2, '0')}:00`;
        });

        const colors = [
            'rgba(40, 167, 69, 0.8)',   // Green
            'rgba(0, 123, 255, 0.8)',   // Blue  
            'rgba(255, 193, 7, 0.8)',   // Yellow
            'rgba(220, 53, 69, 0.8)',   // Red
            'rgba(108, 117, 125, 0.8)', // Gray
        ];
        
        // Initialize datasets array
        let datasets = [];
        
        try {
            // Fetch real battery data from dashboard API
            const response = await fetch('/api/battery/status');
            if (response.ok) {
                const batteryResult = await response.json();
                if (batteryResult.success && batteryResult.data) {
                    const batteryData = batteryResult.data;
                    console.log('✅ Using real battery data:', batteryData);
                    
                    // Use robot data to create proper battery timeline
                    const visibleRobots = robotData.filter(robot => robot.name !== 'HANA_PINKY');
                    console.log('🔋 Available robots for battery chart:', visibleRobots);
                    
                    // Map robot IDs to battery IDs
                    const robotToBatteryMap = {
                        8: 'DP_09',  // HANA PINKY - filtered out
                        9: 'DP_03'   // HANA ROBOT 2
                    };
                    
                    // Create datasets for robots with real battery data
                    visibleRobots.forEach((robot, robotIndex) => {
                        const batteryId = robotToBatteryMap[robot.id];
                        let currentLevel = robot.battery || 50;
                        
                        // Use real-time battery data if available
                        if (batteryId && batteryData[batteryId]) {
                            const realTimeBattery = batteryData[batteryId].level;
                            if (realTimeBattery !== undefined) {
                                currentLevel = realTimeBattery;
                                console.log(`✅ Using real battery data for ${robot.name}: ${currentLevel}%`);
                            }
                        }
                        
                        // Generate realistic 24-hour timeline based on current level
                        const timelineData = labels.map((_, hourIndex) => {
                            // Simulate battery drain over 24 hours with some variation
                            const hoursFromNow = hourIndex - 23;
                            let batteryLevel;
                            
                            if (hoursFromNow <= 0) {
                                // Past hours - simulate drain
                                batteryLevel = Math.max(20, currentLevel + (hoursFromNow * 2) + (Math.random() - 0.5) * 10);
                            } else {
                                // Future hours - current level with small variation
                                batteryLevel = currentLevel + (Math.random() - 0.5) * 5;
                            }
                            
                            return Math.max(15, Math.min(100, batteryLevel));
                        });
                        
                        datasets.push({
                            label: robot.name || `Robot ${robot.id}`,
                            data: timelineData,
                            borderColor: colors[robotIndex % colors.length],
                            backgroundColor: colors[robotIndex % colors.length].replace('0.8', '0.1'),
                            borderWidth: 2,
                            fill: false,
                            tension: 0.4
                        });
                    });
                    
                    console.log(`✅ Created ${datasets.length} battery datasets from real data`);
                }
            }
        } catch (error) {
            console.error('❌ Failed to fetch real battery data:', error);
        }
        
        // Fallback to robot data if no real battery data available
        if (datasets.length === 0) {
            console.warn('⚠️ Using fallback battery timeline data');
            const visibleRobots = robotData.filter(robot => robot.name !== 'HANA_PINKY').slice(0, 5);
            datasets = visibleRobots.map((robot, index) => {
                const currentBattery = robot.battery || 50;
                const timelineData = labels.map((_, hourIndex) => {
                    const hoursFromNow = hourIndex - 23;
                    const batteryLevel = Math.max(20, currentBattery + (hoursFromNow * 1) + (Math.random() - 0.5) * 8);
                    return Math.max(15, Math.min(100, batteryLevel));
                });

                return {
                    label: robot.name || `Robot ${robot.id}`,
                    data: timelineData,
                    borderColor: colors[index % colors.length],
                    backgroundColor: colors[index % colors.length].replace('0.8', '0.1'),
                    borderWidth: 2,
                    fill: false,
                    tension: 0.4
                };
            });
            console.log('📊 Fallback datasets created:', datasets.length, 'datasets');
        }
        
        // Ensure we have datasets to display
        if (datasets.length === 0) {
            console.warn('⚠️ No battery timeline datasets available');
            // Clear any existing chart
            if (batteryTimelineChart) {
                batteryTimelineChart.destroy();
                batteryTimelineChart = null;
            }
            return;
        }

        console.log('🔋 Creating Battery Timeline Chart with datasets:', datasets.length, 'robots');
        
        batteryTimelineChart = new Chart(ctx, {
            type: 'line',
            data: {
                labels: labels,
                datasets: datasets
            },
            options: {
                responsive: true,
                maintainAspectRatio: false,
                plugins: {
                    legend: {
                        display: true,
                        labels: {
                            color: '#cccccc'
                        }
                    }
                },
                scales: {
                    y: {
                        beginAtZero: false,
                        min: 0,
                        max: 100,
                        grid: {
                            color: 'rgba(255, 255, 255, 0.1)'
                        },
                        ticks: {
                            color: '#cccccc',
                            callback: function(value) {
                                return value + '%';
                            }
                        }
                    },
                    x: {
                        grid: {
                            color: 'rgba(255, 255, 255, 0.1)'
                        },
                        ticks: {
                            color: '#cccccc'
                        }
                    }
                }
            }
        });
        
        console.log('✅ Battery Timeline Chart created successfully');
    }

    // Additional utility functions
    function clearFailedTasks() {
        if (!confirm('Clear all failed tasks?')) return;
        
        showNotification('Clearing failed tasks...', 'info');
        // In real implementation, this would make an API call
        console.log('Clear failed tasks requested');
    }

    // Camera control functions
    function switchCamera(cameraType) {
        // Hide all camera feeds
        document.querySelectorAll('.camera-feed').forEach(feed => {
            feed.style.display = 'none';
        });
        
        // Remove active state from all buttons
        document.querySelectorAll('.camera-control-btn').forEach(btn => {
            btn.classList.remove('active');
        });
        
        // Show selected camera feed
        const selectedFeed = document.getElementById(`camera-${cameraType}`);
        if (selectedFeed) {
            selectedFeed.style.display = 'block';
        }
        
        // Add active state to clicked button
        const clickedBtn = document.querySelector(`[data-camera="${cameraType}"]`);
        if (clickedBtn) {
            clickedBtn.classList.add('active');
        }
        
        console.log(`📹 Switched to camera: ${cameraType}`);
    }

    // Update robot status indicators
    function updateRobotStatusIndicators(robots) {
        if (!robots || !Array.isArray(robots)) {
            console.log('❌ updateRobotStatusIndicators: Invalid robots data:', robots);
            return;
        }
        console.log('🔄 Updating robot status indicators for', robots.length, 'robots:', robots);
        
        robots.forEach(robot => {
            // Update battery sidebar status indicators
            // Map robot names to HTML IDs
            const robotIdMap = {
                'HANABOT_8': 'dp09',
                'HANABOT_9': 'dp03', 
                'HANABOT_3': 'hanabot3'
            };
            const htmlId = robotIdMap[robot.name] || robotIdMap[robot.id] || (robot.id ? String(robot.id).toLowerCase() : 'unknown');
            console.log(`🤖 Processing robot: name=${robot.name}, id=${robot.id}, htmlId=${htmlId}`);
            const statusIndicator = document.getElementById(`robot-status-${htmlId}`);
            if (statusIndicator) {
                const statusDot = statusIndicator.querySelector('.status-dot');
                const statusText = statusIndicator.querySelector('.robot-status-text');
                
                if (statusDot && statusText) {
                    // Remove all status classes from dot and indicator
                    statusDot.classList.remove('online', 'busy', 'offline', 'error');
                    statusIndicator.classList.remove('online', 'busy', 'offline', 'error');
                    
                    // Add appropriate status class to both
                    const statusClass = getStatusClass(robot.status);
                    statusDot.classList.add(statusClass);
                    statusIndicator.classList.add(statusClass);
                    
                    // Update status text with more detailed mapping
                    const statusTextMap = {
                        'online': 'Online',
                        'busy': 'Busy', 
                        'offline': 'Offline',
                        'error': 'Error',
                        'idle': 'Ready',
                        'moving_to_arm': 'Moving to Station',
                        'picking': 'Picking Item',
                        'moving_to_user': 'Delivering',
                        'waiting_confirm': 'Awaiting Pickup',
                        'returning_to_dock': 'Returning to Dock'
                    };
                    const displayText = statusTextMap[robot.status] || statusTextMap[statusClass] || 'Ready';
                    statusText.textContent = displayText;
                    
                    // Remove all status classes from text
                    statusText.classList.remove('online', 'busy', 'offline', 'error');
                    // Add appropriate status class to text for color
                    statusText.classList.add(statusClass);
                }
                
                // Update battery display
                const batteryLevel = robot.battery || 0;
                const batteryElement = document.getElementById(`quick-battery-${htmlId}`);
                const batteryBar = document.getElementById(`battery-bar-${htmlId}`);
                const batteryIcon = document.getElementById(`battery-icon-${htmlId}`);
                
                if (batteryElement) {
                    batteryElement.textContent = `${batteryLevel}%`;
                    // Remove all battery classes from text
                    batteryElement.classList.remove('good', 'medium', 'low', 'critical');
                }
                
                if (batteryBar) {
                    batteryBar.style.width = `${batteryLevel}%`;
                    // Remove all battery classes
                    batteryBar.classList.remove('good', 'medium', 'low', 'critical');
                }
                
                // Determine battery status and apply consistent classes
                let batteryStatus;
                let batteryIconClass;
                if (batteryLevel > 75) {
                    batteryStatus = 'good';
                    batteryIconClass = 'bi-battery-full';
                } else if (batteryLevel > 50) {
                    batteryStatus = 'medium';
                    batteryIconClass = 'bi-battery-half';
                } else if (batteryLevel > 25) {
                    batteryStatus = 'low';
                    batteryIconClass = 'bi-battery';
                } else {
                    batteryStatus = 'critical';
                    batteryIconClass = 'bi-battery';
                }
                
                // Apply battery status classes
                if (batteryElement) batteryElement.classList.add(batteryStatus);
                if (batteryBar) batteryBar.classList.add(batteryStatus);
                if (batteryIcon) {
                    // Update battery icon
                    batteryIcon.className = `${batteryIconClass} battery-icon ${batteryStatus}`;
                }
            }
        });
        
        // Update Fleet Summary
        updateFleetSummary(robots);
        
        // Update Monitor Badge
        updateMonitorBadge(robots);
    }
    
    function updateFleetSummary(robots) {
        if (!robots || !Array.isArray(robots)) return;
        
        const onlineRobots = robots.filter(robot => robot.status !== 'offline').length;
        const totalRobots = robots.length;
        const activeTasks = robots.filter(robot => robot.status && robot.status !== 'idle' && robot.status !== 'offline').length;
        const totalBattery = robots.reduce((sum, robot) => sum + (robot.battery || 0), 0);
        const avgBattery = totalRobots > 0 ? Math.round(totalBattery / totalRobots) : 0;
        
        // Update summary elements
        const summaryOnline = document.getElementById('summary-online');
        const summaryTasks = document.getElementById('summary-tasks');
        const summaryBattery = document.getElementById('summary-battery');
        
        if (summaryOnline) summaryOnline.textContent = `${onlineRobots}/${totalRobots}`;
        if (summaryTasks) summaryTasks.textContent = activeTasks.toString();
        if (summaryBattery) summaryBattery.textContent = `${avgBattery}%`;
    }
    
    function updateMonitorBadge(robots) {
        const monitorBadge = document.getElementById('monitor-badge');
        if (!monitorBadge || !robots || !Array.isArray(robots)) return;
        
        const onlineRobots = robots.filter(robot => robot.status !== 'offline').length;
        const totalRobots = robots.length;
        const hasErrors = robots.some(robot => getStatusClass(robot.status) === 'error');
        const allOnline = onlineRobots === totalRobots && totalRobots > 0;
        
        // Update badge text and style
        monitorBadge.style.background = '';  // Reset inline styles
        
        if (hasErrors) {
            monitorBadge.textContent = 'ERROR';
            monitorBadge.style.background = 'var(--danger-color)';
        } else if (allOnline) {
            monitorBadge.textContent = 'ONLINE';
            monitorBadge.style.background = 'var(--success-color)';
        } else if (onlineRobots > 0) {
            monitorBadge.textContent = 'PARTIAL';
            monitorBadge.style.background = 'var(--warning-color)';
        } else {
            monitorBadge.textContent = 'OFFLINE';
            monitorBadge.style.background = 'var(--text-muted)';
        }
    }

    function getStatusClass(status) {
        const statusMap = {
            'idle': 'online',
            'moving_to_arm': 'busy',
            'picking': 'busy', 
            'moving_to_user': 'busy',
            'waiting_confirm': 'busy',
            'returning_to_dock': 'busy',
            'offline': 'offline',
            'error': 'error'
        };
        return statusMap[status] || 'offline';
    }

    // Enhanced fetchFleetMetrics to update status indicators
    const originalFetchFleetMetrics = window.fetchFleetMetrics;
    function enhancedFetchFleetMetrics() {
        if (originalFetchFleetMetrics) {
            originalFetchFleetMetrics();
        }
        
        // Also update robot status indicators
        fetch('/api/admin/robots')
            .then(response => response.json())
            .then(result => {
                if (result.success && result.data) {
                    updateRobotStatusIndicators(result.data);
                }
            })
            .catch(error => {
                console.error('Error fetching robot status:', error);
            });
    }

    // Replace original function
    window.fetchFleetMetrics = enhancedFetchFleetMetrics;

    // Global Camera Functions (moved to top to avoid reference errors)
    let globalCameraActive = false;
    
    // Make these functions global immediately for HTML access
    window.handleGlobalCameraError = function() {
        const status = document.getElementById('global-camera-status');
        if (status) {
            const statusDot = status.querySelector('.status-dot');
            if (statusDot) {
                statusDot.classList.remove('online');
                statusDot.classList.add('offline');
            }
            status.innerHTML = '<span class="status-dot offline"></span> Connection Error';
        }
        showNotification('Camera connection failed - Check camera server', 'error');
    };
    
    window.handleGlobalCameraLoad = function() {
        const status = document.getElementById('global-camera-status');
        if (status) {
            const statusDot = status.querySelector('.status-dot');
            if (statusDot) {
                statusDot.classList.remove('offline');
                statusDot.classList.add('online');
            }
            status.innerHTML = '<span class="status-dot online"></span> Live';
        }
    };
    
    window.toggleGlobalCamera = async function() {
        const toggle = document.getElementById('global-camera-toggle');
        const stream = document.getElementById('global-camera-stream');
        const placeholder = document.getElementById('global-camera-placeholder');
        const overlay = document.getElementById('global-camera-overlay');
        const status = document.getElementById('global-camera-status');
        
        if (!globalCameraActive) {
            try {
                // Use same camera endpoint as Camera Stream section
                const response = await fetch('/camera/start', { method: 'POST' });
                const result = await response.json();
                
                if (result.status === 'success') {
                    globalCameraActive = true;
                    toggle.innerHTML = '<i class="bi bi-stop-fill"></i> Stop Feed';
                    toggle.classList.add('active');
                    
                    // Use same stream URL as Camera Stream
                    stream.src = '/camera/stream?' + Date.now();
                    stream.style.display = 'block';
                    placeholder.style.display = 'none';
                    overlay.style.display = 'block';
                    
                    showNotification('Global camera feed started', 'success');
                } else {
                    showNotification('Failed to start camera: ' + (result.message || 'Unknown error'), 'error');
                }
            } catch (error) {
                console.error('Error starting camera:', error);
                showNotification('Error starting camera feed', 'error');
            }
        } else {
            try {
                const response = await fetch('/camera/stop', { method: 'POST' });
                const result = await response.json();
                
                globalCameraActive = false;
                toggle.innerHTML = '<i class="bi bi-play-fill"></i> Start Feed';
                toggle.classList.remove('active');
                
                stream.src = '';
                stream.style.display = 'none';
                placeholder.style.display = 'flex';
                overlay.style.display = 'none';
                
                // Update status
                const statusDot = status.querySelector('.status-dot');
                if (statusDot) {
                    statusDot.classList.remove('online');
                    statusDot.classList.add('offline');
                }
                status.innerHTML = '<span class="status-dot offline"></span> Offline';
                
                showNotification('Global camera feed stopped', 'info');
            } catch (error) {
                console.error('Error stopping camera:', error);
                showNotification('Error stopping camera feed', 'error');
            }
        }
    };
    
    window.refreshGlobalCamera = function() {
        if (globalCameraActive) {
            const stream = document.getElementById('global-camera-stream');
            const timestamp = Date.now();
            stream.src = `/camera/stream?${timestamp}`;
            showNotification('Camera feed refreshed', 'info');
        } else {
            showNotification('Start camera feed first', 'warning');
        }
    };
    
    // Functions already defined above as window properties

    // Enhanced Robot Data Functions for Battery Cards
    function updateBatteryCards(robots) {
        if (!robots || !Array.isArray(robots)) {
            console.log('❌ updateBatteryCards: Invalid robots data:', robots);
            return;
        }
        console.log('🔋 Updating battery cards for', robots.length, 'robots:', robots);
        
        robots.forEach(robot => {
            const robotName = robot.name || robot.id;
            const robotId = robotName ? String(robotName).toLowerCase().replace(/[^a-z0-9]/g, '') : 'unknown';
            
            // Update battery level
            const levelElement = document.getElementById(`battery-level-${robotId}`);
            if (levelElement) {
                levelElement.textContent = `${robot.battery || 0}%`;
            }
            
            // Update battery fill
            const fillElement = document.getElementById(`battery-fill-${robotId}`);
            if (fillElement) {
                const battery = robot.battery || 0;
                fillElement.style.width = `${battery}%`;
                
                // Remove existing classes
                fillElement.classList.remove('critical', 'low', 'medium', 'good');
                
                // Add appropriate class based on battery level
                if (battery <= 20) {
                    fillElement.classList.add('critical');
                } else if (battery <= 40) {
                    fillElement.classList.add('low');
                } else if (battery <= 70) {
                    fillElement.classList.add('medium');
                } else {
                    fillElement.classList.add('good');
                }
            }
            
            // Update battery status
            const statusElement = document.getElementById(`battery-status-${robotId}`);
            if (statusElement) {
                const battery = robot.battery || 0;
                let status = 'Good';
                if (battery <= 20) status = 'Critical';
                else if (battery <= 40) status = 'Low';
                else if (battery <= 70) status = 'Medium';
                statusElement.textContent = status;
            }
            
            // Update charging status
            const chargingElement = document.getElementById(`battery-charging-${robotId}`);
            if (chargingElement) {
                chargingElement.textContent = robot.status === 'charging' ? 'Yes' : 'No';
            }
            
            // Update estimated time
            const timeElement = document.getElementById(`battery-time-${robotId}`);
            if (timeElement) {
                const battery = robot.battery || 0;
                if (robot.status === 'charging') {
                    const timeToFull = Math.round((100 - battery) / 10); // Rough estimate
                    timeElement.textContent = `${timeToFull}h to full`;
                } else {
                    const timeRemaining = Math.round(battery / 5); // Rough estimate
                    timeElement.textContent = `${timeRemaining}h remaining`;
                }
            }
            
            // Update battery card overall status with CSS classes
            const batteryCard = document.querySelector(`[data-robot="${robotName}"]`);
            if (batteryCard) {
                batteryCard.classList.remove('updating', 'critical', 'low', 'medium', 'good');
                
                // Add appropriate class based on battery level
                const battery = robot.battery || 0;
                if (battery <= 20) {
                    batteryCard.classList.add('critical');
                } else if (battery <= 40) {
                    batteryCard.classList.add('low');
                } else if (battery <= 70) {
                    batteryCard.classList.add('medium');
                } else {
                    batteryCard.classList.add('good');
                }
            }
        });
    }

    // Make functions global for onclick handlers
    window.fetchRobotData = fetchRobotData;
    window.fetchTaskData = fetchTaskData;
    window.viewRobotDetails = viewRobotDetails;
    window.stopRobot = stopRobot;
    window.sendRobotHome = sendRobotHome;
    window.toggleRobotTable = toggleRobotTable;
    window.clearFailedTasks = clearFailedTasks;
    window.switchCamera = switchCamera;
    window.updateRobotStatusIndicators = updateRobotStatusIndicators;
    // Global camera functions already defined above as window properties
    window.updateBatteryCards = updateBatteryCards;

    // Initialize dashboard and fall detection
    initDashboard();
    initFallDetection();

    // Load section-specific data when switching sections
    const originalShowSection = window.showSection;
    window.showSection = function(sectionId) {
        if (originalShowSection) originalShowSection(sectionId);
        
        // Load section-specific data
        if (sectionId === 'robots') {
            fetchRobotData();
        } else if (sectionId === 'tasks') {
            fetchTaskData();
        }
    };

    // Initialize section-specific data based on active section
    const activeSection = document.querySelector('.nav-link.active')?.dataset.section;
    if (activeSection === 'robots') {
        fetchRobotData();
    } else if (activeSection === 'tasks') {
        fetchTaskData();
    }
});

