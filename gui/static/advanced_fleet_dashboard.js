document.addEventListener('DOMContentLoaded', () => {
    let updateInterval = null;
    let charts = {};
    let currentData = null;

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

    // Initialize charts
    function initCharts() {
        // Hourly Distribution Chart
        const hourlyCtx = document.getElementById('hourlyChart').getContext('2d');
        charts.hourly = new Chart(hourlyCtx, {
            type: 'line',
            data: {
                labels: Array.from({length: 7}, (_, i) => {
                    const date = new Date();
                    date.setDate(date.getDate() - (6 - i));
                    return date.toLocaleDateString('ko-KR', { month: 'short', day: 'numeric' });
                }),
                datasets: [{
                    label: 'Total Tasks',
                    data: [],
                    borderColor: colors.primary,
                    backgroundColor: colors.primary + '20',
                    fill: true,
                    tension: 0.4
                }, {
                    label: 'Completed',
                    data: [],
                    borderColor: colors.success,
                    backgroundColor: colors.success + '20',
                    fill: true,
                    tension: 0.4
                }, {
                    label: 'Failed',
                    data: [],
                    borderColor: colors.danger,
                    backgroundColor: colors.danger + '20',
                    fill: true,
                    tension: 0.4
                }]
            },
            options: {
                responsive: true,
                maintainAspectRatio: false,
                scales: {
                    y: {
                        beginAtZero: true,
                        grid: {
                            color: '#333333'
                        }
                    },
                    x: {
                        grid: {
                            color: '#333333'
                        }
                    }
                },
                plugins: {
                    legend: {
                        display: true,
                        position: 'top'
                    }
                }
            }
        });

        // Status Distribution Chart
        const statusCtx = document.getElementById('statusChart').getContext('2d');
        charts.status = new Chart(statusCtx, {
            type: 'doughnut',
            data: {
                labels: ['Completed', 'Failed', 'Active', 'Pending'],
                datasets: [{
                    data: [0, 0, 0, 0],
                    backgroundColor: [
                        colors.success,
                        colors.danger,
                        colors.warning,
                        colors.muted
                    ],
                    borderWidth: 2,
                    borderColor: '#1e1e1e'
                }]
            },
            options: {
                responsive: true,
                maintainAspectRatio: false,
                plugins: {
                    legend: {
                        position: 'bottom'
                    }
                }
            }
        });

        // Robot Performance Chart
        const robotCtx = document.getElementById('robotChart').getContext('2d');
        charts.robot = new Chart(robotCtx, {
            type: 'bar',
            data: {
                labels: [],
                datasets: [{
                    label: 'Completed Tasks',
                    data: [],
                    backgroundColor: colors.success + '80',
                    borderColor: colors.success,
                    borderWidth: 1
                }, {
                    label: 'Failed Tasks',
                    data: [],
                    backgroundColor: colors.danger + '80',
                    borderColor: colors.danger,
                    borderWidth: 1
                }]
            },
            options: {
                responsive: true,
                maintainAspectRatio: false,
                scales: {
                    y: {
                        beginAtZero: true,
                        grid: {
                            color: '#333333'
                        }
                    },
                    x: {
                        grid: {
                            color: '#333333'
                        }
                    }
                }
            }
        });

        // Trend Chart
        const trendCtx = document.getElementById('trendChart').getContext('2d');
        charts.trend = new Chart(trendCtx, {
            type: 'line',
            data: {
                labels: [],
                datasets: [{
                    label: 'Daily Tasks',
                    data: [],
                    borderColor: colors.primary,
                    backgroundColor: colors.primary + '20',
                    fill: true,
                    tension: 0.4
                }, {
                    label: 'Success Rate %',
                    data: [],
                    borderColor: colors.success,
                    backgroundColor: colors.success + '20',
                    fill: false,
                    yAxisID: 'percentage',
                    tension: 0.4
                }]
            },
            options: {
                responsive: true,
                maintainAspectRatio: false,
                scales: {
                    y: {
                        beginAtZero: true,
                        grid: {
                            color: '#333333'
                        }
                    },
                    percentage: {
                        type: 'linear',
                        display: true,
                        position: 'right',
                        min: 0,
                        max: 100,
                        grid: {
                            drawOnChartArea: false,
                        },
                    },
                    x: {
                        grid: {
                            color: '#333333'
                        }
                    }
                }
            }
        });
    }

    // Update metrics display
    function updateMetrics(metrics) {
        document.getElementById('total-tasks-today').textContent = metrics.total_tasks_today || 0;
        
        const avgTime = metrics.avg_completion_time;
        document.getElementById('avg-completion-time').textContent = 
            (avgTime && typeof avgTime === 'number') ? avgTime.toFixed(1) : '0.0';
        
        document.getElementById('active-users').textContent = metrics.unique_users_today || 0;
        document.getElementById('active-robots').textContent = metrics.active_robots_today || 0;
        
        const successRate = (metrics.total_tasks_today && metrics.total_tasks_today > 0) 
            ? ((metrics.completed_today / metrics.total_tasks_today) * 100).toFixed(1)
            : '0.0';
        document.getElementById('success-rate').textContent = successRate + '%';
        document.getElementById('failed-tasks').textContent = metrics.failed_today || 0;

        // Update change indicators (mock data for demo)
        updateChangeIndicator('tasks-change', 12, 'positive');
        updateChangeIndicator('time-change', -5, 'positive');
        updateChangeIndicator('users-change', 3, 'positive');
        updateChangeIndicator('success-change', 2.3, 'positive');
    }

    function updateChangeIndicator(elementId, change, type) {
        const element = document.getElementById(elementId);
        const prefix = change > 0 ? '+' : '';
        const suffix = elementId.includes('change') && !elementId.includes('users') ? '%' : '';
        
        element.textContent = `${prefix}${change}${suffix}`;
        element.className = `metric-change ${type}`;
    }

    // Update hourly chart (now shows 7 days by default)
    function updateHourlyChart(hourlyData) {
        // Default to 7 days view
        const taskCounts = new Array(7).fill(0);
        const completedCounts = new Array(7).fill(0);
        const failedCounts = new Array(7).fill(0);

        // Generate mock data for 7 days (replace with real data later)
        for (let i = 0; i < 7; i++) {
            taskCounts[i] = Math.floor(Math.random() * 50) + 10;
            completedCounts[i] = Math.floor(taskCounts[i] * (0.7 + Math.random() * 0.2));
            failedCounts[i] = taskCounts[i] - completedCounts[i];
        }

        charts.hourly.data.datasets[0].data = taskCounts;
        charts.hourly.data.datasets[1].data = completedCounts;
        charts.hourly.data.datasets[2].data = failedCounts;
        charts.hourly.update();
    }

    // Update status chart
    function updateStatusChart(metrics) {
        const data = [
            metrics.completed_today || 0,
            metrics.failed_today || 0,
            metrics.active_today || 0,
            (metrics.total_tasks_today || 0) - (metrics.completed_today || 0) - (metrics.failed_today || 0) - (metrics.active_today || 0)
        ];
        
        charts.status.data.datasets[0].data = data;
        charts.status.update();
    }

    // Update robot performance chart  
    function updateRobotChart(robotData) {
        const names = robotData.map(robot => robot.bot_name || `Robot ${robot.hana_bot_id}`);
        const completed = robotData.map(robot => robot.completed_tasks || 0);
        const failed = robotData.map(robot => robot.failed_tasks || 0);

        charts.robot.data.labels = names;
        charts.robot.data.datasets[0].data = completed;
        charts.robot.data.datasets[1].data = failed;
        charts.robot.update();
    }

    // Update trend chart
    function updateTrendChart(trendData) {
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

        charts.trend.data.labels = dates;
        charts.trend.data.datasets[0].data = tasks;
        charts.trend.data.datasets[1].data = successRates;
        charts.trend.update();
    }

    // Update data tables
    function updateTopUsersTable(userData) {
        const tbody = document.getElementById('top-users-table');
        
        if (!userData || userData.length === 0) {
            tbody.innerHTML = '<tr><td colspan="5" class="text-center text-muted">No data available</td></tr>';
            return;
        }

        let html = '';
        userData.forEach((user, index) => {
            const successRate = user.order_count > 0 
                ? ((user.completed_orders / user.order_count) * 100).toFixed(1)
                : '0.0';
            
            const performanceClass = successRate >= 90 ? 'excellent' : 
                                   successRate >= 70 ? 'good' : 'poor';

            html += `
                <tr>
                    <td>
                        <strong>${user.name}</strong><br>
                        <small class="text-muted">ID: ${user.resident_id}</small>
                    </td>
                    <td><strong>${user.order_count}</strong></td>
                    <td>${user.completed_orders}</td>
                    <td>${successRate}%</td>
                    <td>
                        <div class="performance-bar">
                            <div class="performance-fill ${performanceClass}" style="width: ${successRate}%"></div>
                        </div>
                    </td>
                </tr>
            `;
        });

        tbody.innerHTML = html;
    }

    function updatePopularItemsTable(itemData) {
        const tbody = document.getElementById('popular-items-table');
        
        if (!itemData || itemData.length === 0) {
            tbody.innerHTML = '<tr><td colspan="5" class="text-center text-muted">No data available</td></tr>';
            return;
        }

        let html = '';
        const maxOrders = Math.max(...itemData.map(item => item.order_count));
        
        itemData.forEach((item, index) => {
            const successRate = item.order_count > 0 
                ? ((item.delivered_count / item.order_count) * 100).toFixed(1)
                : '0.0';
            
            const popularity = maxOrders > 0 
                ? ((item.order_count / maxOrders) * 100).toFixed(0)
                : 0;

            html += `
                <tr>
                    <td>
                        <strong>${item.item_type}</strong><br>
                        <small class="text-muted">ID: ${item.item_id}</small>
                    </td>
                    <td><strong>${item.order_count}</strong></td>
                    <td>${item.delivered_count}</td>
                    <td>${successRate}%</td>
                    <td>
                        <div class="performance-bar">
                            <div class="performance-fill excellent" style="width: ${popularity}%"></div>
                        </div>
                    </td>
                </tr>
            `;
        });

        tbody.innerHTML = html;
    }

    // Fetch analytics data
    async function fetchAnalytics() {
        try {
            const response = await fetch('/api/fleet/analytics');
            
            if (!response.ok) {
                throw new Error(`HTTP ${response.status}: ${response.statusText}`);
            }
            
            const result = await response.json();

            if (result.success) {
                currentData = result.data;
                
                // Update all components
                updateMetrics(currentData.system_metrics);
                updateHourlyChart(currentData.hourly_distribution);
                updateStatusChart(currentData.system_metrics);
                updateRobotChart(currentData.robot_performance);
                updateTrendChart(currentData.daily_trends);
                updateTopUsersTable(currentData.top_requesters);
                updatePopularItemsTable(currentData.popular_items);

            } else {
                throw new Error(result.error || 'Unknown API error');
            }
        } catch (error) {
            console.error('Failed to fetch analytics:', error);
            showNotification('Failed to load analytics data: ' + error.message, 'error');
        }
    }

    // Control functions
    async function sendControlCommand(action, targetId = null) {
        try {
            const response = await fetch('/api/fleet/control', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify({
                    action: action,
                    target_id: targetId
                })
            });

            const result = await response.json();
            
            if (result.success) {
                showNotification(result.message, 'success');
                // Refresh data after control action
                setTimeout(fetchAnalytics, 1000);
            } else {
                showNotification('Control action failed: ' + result.error, 'error');
            }
        } catch (error) {
            console.error('Control command failed:', error);
            showNotification('Control command failed: ' + error.message, 'error');
        }
    }

    // Notification system
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

    // Global functions for HTML onclick handlers
    window.refreshData = () => {
        showNotification('Refreshing analytics data...', 'info');
        fetchAnalytics();
    };

    window.exportData = () => {
        if (!currentData) {
            showNotification('No data available to export', 'error');
            return;
        }
        
        const dataStr = JSON.stringify(currentData, null, 2);
        const dataBlob = new Blob([dataStr], {type: 'application/json'});
        const url = URL.createObjectURL(dataBlob);
        
        const link = document.createElement('a');
        link.href = url;
        link.download = `fleet_analytics_${new Date().toISOString().split('T')[0]}.json`;
        link.click();
        
        URL.revokeObjectURL(url);
        showNotification('Analytics data exported successfully', 'success');
    };

    window.emergencyStop = () => {
        if (confirm('Are you sure you want to issue an emergency stop to all robots?')) {
            sendControlCommand('emergency_stop');
        }
    };

    window.resetSystem = () => {
        if (confirm('Reset all robot statuses to idle?')) {
            sendControlCommand('reset_system');
        }
    };

    window.optimizeRoutes = () => {
        showNotification('Route optimization initiated...', 'info');
        sendControlCommand('optimize_routes');
    };

    window.maintenanceMode = () => {
        if (confirm('Enable maintenance mode? This will pause all operations.')) {
            sendControlCommand('maintenance_mode');
        }
    };

    window.generateReport = () => {
        showNotification('Generating comprehensive fleet report...', 'info');
        // This would generate a PDF report in a real implementation
        setTimeout(() => {
            showNotification('Report generation completed', 'success');
        }, 3000);
    };

    window.clearFailedTasks = () => {
        if (confirm('Clear all failed tasks from the database?')) {
            sendControlCommand('fail_all_incomplete');
        }
    };

    // Initialize dashboard
    async function initDashboard() {
        console.log('Advanced Fleet Analytics Dashboard initialized');
        
        // Initialize charts
        initCharts();
        
        // Initial data fetch
        await fetchAnalytics();
        
        // Set up auto-refresh every 10 seconds
        updateInterval = setInterval(fetchAnalytics, 10000);
        
        showNotification('Advanced Fleet Analytics Dashboard loaded', 'success');
    }

    // Cleanup on page unload
    window.addEventListener('beforeunload', () => {
        if (updateInterval) {
            clearInterval(updateInterval);
        }
    });

    // Chart period controls
    document.addEventListener('click', (e) => {
        if (e.target.classList.contains('chart-control')) {
            // Remove active class from siblings
            e.target.parentElement.querySelectorAll('.chart-control').forEach(btn => {
                btn.classList.remove('active');
            });
            
            // Add active class to clicked button
            e.target.classList.add('active');
            
            // Trigger different data fetching based on period
            const period = e.target.dataset.period;
            showNotification(`Switching to ${period} view...`, 'info');
            
            // Update chart data based on period
            updateChartPeriod(period);
        }
    });

    // Update chart based on time period
    function updateChartPeriod(period) {
        let labels, dataPoints;
        
        switch(period) {
            case '24h':
                labels = Array.from({length: 24}, (_, i) => `${i}:00`);
                dataPoints = 24;
                break;
            case '7d':
                labels = Array.from({length: 7}, (_, i) => {
                    const date = new Date();
                    date.setDate(date.getDate() - (6 - i));
                    return date.toLocaleDateString('ko-KR', { month: 'short', day: 'numeric' });
                });
                dataPoints = 7;
                break;
            case '30d':
                labels = Array.from({length: 30}, (_, i) => {
                    const date = new Date();
                    date.setDate(date.getDate() - (29 - i));
                    return date.toLocaleDateString('ko-KR', { month: 'short', day: 'numeric' });
                });
                dataPoints = 30;
                break;
            default:
                return;
        }
        
        // Update hourly chart
        if (charts.hourly) {
            charts.hourly.data.labels = labels;
            
            // Generate mock data for different periods
            const totalTasks = Array.from({length: dataPoints}, () => Math.floor(Math.random() * 20) + 5);
            const completedTasks = totalTasks.map(total => Math.floor(total * (0.7 + Math.random() * 0.2)));
            const failedTasks = totalTasks.map((total, i) => total - completedTasks[i]);
            
            charts.hourly.data.datasets[0].data = totalTasks;
            charts.hourly.data.datasets[1].data = completedTasks;
            charts.hourly.data.datasets[2].data = failedTasks;
            
            charts.hourly.update();
        }
        
        // Update chart title
        const chartTitle = document.querySelector('.chart-section h3');
        if (chartTitle && chartTitle.textContent.includes('Hourly Task Distribution')) {
            const periods = {'24h': '24h', '7d': '7 days', '30d': '30 days'};
            chartTitle.textContent = `Hourly Task Distribution (${periods[period]})`;
        }
    }

    // Start dashboard
    initDashboard();
});