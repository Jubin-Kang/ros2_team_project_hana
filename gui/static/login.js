document.addEventListener('DOMContentLoaded', () => {
    const form = document.getElementById('login-form');
    const loginButton = document.getElementById('login-button');
    const loginBtnText = document.getElementById('login-btn-text');
    const errorMessage = document.getElementById('error-message');
    const successMessage = document.getElementById('success-message');
    const loginIdInput = document.getElementById('login_id');
    const passwordInput = document.getElementById('password');

    // 입력 필드 포커스 효과
    [loginIdInput, passwordInput].forEach(input => {
        input.addEventListener('focus', () => {
            input.parentElement.classList.add('focused');
        });
        
        input.addEventListener('blur', () => {
            if (!input.value) {
                input.parentElement.classList.remove('focused');
            }
        });
    });

    // 메시지 표시 함수
    function showMessage(type, text) {
        // 기존 메시지 숨기기
        errorMessage.style.display = 'none';
        successMessage.style.display = 'none';
        
        if (type === 'error') {
            errorMessage.textContent = text;
            errorMessage.style.display = 'block';
        } else if (type === 'success') {
            successMessage.textContent = text;
            successMessage.style.display = 'block';
        }
        
        // 5초 후 자동 숨김
        setTimeout(() => {
            errorMessage.style.display = 'none';
            successMessage.style.display = 'none';
        }, 5000);
    }

    // 성공 알림 표시
    function showSuccessNotification(message) {
        const notification = document.createElement('div');
        notification.style.cssText = `
            position: fixed;
            top: 20px;
            left: 50%;
            transform: translateX(-50%);
            background: linear-gradient(135deg, #4CAF50, #45a049);
            color: white;
            padding: 15px 25px;
            border-radius: 10px;
            box-shadow: 0 4px 15px rgba(76, 175, 80, 0.3);
            z-index: 9999;
            font-weight: 600;
            animation: slideDown 0.3s ease-out;
        `;
        notification.textContent = message;
        
        // 애니메이션 CSS 추가
        const style = document.createElement('style');
        style.textContent = `
            @keyframes slideDown {
                from { transform: translateX(-50%) translateY(-100%); opacity: 0; }
                to { transform: translateX(-50%) translateY(0); opacity: 1; }
            }
        `;
        document.head.appendChild(style);
        
        document.body.appendChild(notification);
        
        // 3초 후 제거
        setTimeout(() => {
            notification.remove();
            style.remove();
        }, 3000);
    }

    // 폼 제출 처리
    form.addEventListener('submit', async (e) => {
        e.preventDefault();
        
        const loginId = loginIdInput.value.trim();
        const password = passwordInput.value.trim();

        // 입력 검증
        if (!loginId || !password) {
            showMessage('error', '로그인 ID와 비밀번호를 모두 입력해주세요.');
            return;
        }

        // 로딩 상태 시작
        const originalHTML = loginButton.innerHTML;
        loginButton.innerHTML = '<div class="loading"></div>로그인 중...';
        loginButton.disabled = true;

        try {
            const response = await fetch('/login', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify({ 
                    login_id: loginId, 
                    password: password 
                }),
            });

            const data = await response.json();

            if (response.ok) {
                // 로그인 성공
                localStorage.setItem('resident_id', data.resident_id);
                localStorage.setItem('user_name', data.name);
                localStorage.setItem('user_role', data.role || 'user');
                
                // 성공 메시지 표시
                showMessage('success', `${data.name}님, 환영합니다! 🎉`);
                showSuccessNotification(`${data.name}님, 환영합니다! 🎉`);
                
                // 버튼 텍스트 변경
                loginButton.innerHTML = '<i class="bi bi-check-circle me-2"></i>로그인 성공!';
                
                // 2초 후 페이지 이동
                setTimeout(() => {
                    window.location.href = '/order';
                }, 2000);
                
            } else {
                // 로그인 실패
                showMessage('error', data.error || '로그인에 실패했습니다.');
                
                // 버튼 원래 상태로 복구
                loginButton.innerHTML = originalHTML;
                loginButton.disabled = false;
                
                // 입력 필드 포커스
                if (data.error && data.error.includes('ID')) {
                    loginIdInput.focus();
                } else {
                    passwordInput.focus();
                }
            }
        } catch (error) {
            console.error('Login error:', error);
            showMessage('error', '네트워크 오류가 발생했습니다. 다시 시도해주세요.');
            
            // 버튼 원래 상태로 복구
            loginButton.innerHTML = originalHTML;
            loginButton.disabled = false;
        }
    });

    // Enter 키 처리
    [loginIdInput, passwordInput].forEach(input => {
        input.addEventListener('keypress', (e) => {
            if (e.key === 'Enter') {
                form.dispatchEvent(new Event('submit'));
            }
        });
    });

    // 페이지 로드 시 첫 번째 입력 필드에 포커스
    setTimeout(() => {
        loginIdInput.focus();
    }, 500);

    // 로그인 상태 확인 (이미 로그인된 경우 리다이렉트)
    const residentId = localStorage.getItem('resident_id');
    if (residentId) {
        const userName = localStorage.getItem('user_name');
        showSuccessNotification(`${userName}님, 이미 로그인되어 있습니다.`);
        setTimeout(() => {
            window.location.href = '/order';
        }, 1500);
    }
});