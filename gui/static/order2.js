document.addEventListener('DOMContentLoaded', function() {
    console.log('Order2 page loaded.');

    const submitButton = document.getElementById('submitOrder2');
    if (submitButton) {
        submitButton.addEventListener('click', function() {
            const residentName = document.getElementById('residentName').value;
            const orderItem = document.getElementById('orderItem').value;
            const quantity = document.getElementById('quantity').value;
            const notes = document.getElementById('notes').value;

            const orderData = {
                residentName: residentName,
                orderItem: orderItem,
                quantity: parseInt(quantity),
                notes: notes
            };

            fetch('/api/order2/submit', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json'
                },
                body: JSON.stringify(orderData)
            })
            .then(response => response.json())
            .then(data => {
                console.log('Success:', data);
                alert('주문이 성공적으로 제출되었습니다!');
                // Optionally clear the form or update recent orders list
            })
            .catch((error) => {
                console.error('Error:', error);
                alert('주문 제출 중 오류가 발생했습니다.');
            });
        });
    }
});
