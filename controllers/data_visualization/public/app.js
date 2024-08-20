const ctx = document.getElementById('combinedHistogram').getContext('2d');
const combinedHistogram = new Chart(ctx, {
    type: 'bar',
    data: {
        labels: ['Values'],
        datasets: [
            {
                label: 'p1',
                data: [0],
                backgroundColor: 'rgba(255, 99, 132, 0.5)',
                borderColor: 'rgba(255, 99, 132, 1)',
                borderWidth: 1
            },
            {
                label: 'p2',
                data: [0],
                backgroundColor: 'rgba(54, 162, 235, 0.5)',
                borderColor: 'rgba(54, 162, 235, 1)',
                borderWidth: 1
            },
            {
                label: 'p3',
                data: [0],
                backgroundColor: 'rgba(75, 192, 192, 0.5)',
                borderColor: 'rgba(75, 192, 192, 1)',
                borderWidth: 1
            },
            {
                label: 'p4',
                data: [0],
                backgroundColor: 'rgba(153, 102, 255, 0.5)',
                borderColor: 'rgba(153, 102, 255, 1)',
                borderWidth: 1
            }
        ]
    },
    options: {
        scales: {
            y: {
                beginAtZero: true,
                min: 0,    // Set minimum y value
                max: 30,   // Set maximum y value
                ticks: {
                    stepSize: 5 // Adds ticks every 5 units
                }
            }
        },
        animation: {
            duration: 500  // Lower for more "real-time" updates
        }
    }
});

const ws = new WebSocket(`ws://${window.location.hostname}:65432`);
ws.onmessage = function (event) {
    // console.log("Received data:", event.data);  // Log the Blob object
    if (event.data instanceof Blob) {
        // Create a new FileReader to handle the Blob
        const reader = new FileReader();
        
        // This event handler is triggered once reading is finished
        reader.onload = function() {
            // console.log("Converted Blob to text:", reader.result);  // Log the text from Blob
            try {
                const data = JSON.parse(reader.result);  // Parse the text as JSON
                // console.log("Parsed data:", data);  // Log parsed data
                
                // Update chart with the parsed data
                combinedHistogram.data.datasets[0].data = [data.p1];
                combinedHistogram.data.datasets[1].data = [data.p2];
                combinedHistogram.data.datasets[2].data = [data.p3];
                combinedHistogram.data.datasets[3].data = [data.p4];
                combinedHistogram.update();
            } catch (error) {
                console.error("Error parsing JSON:", error);
            }
        };
        
        // Start reading the blob as text
        reader.readAsText(event.data);
    } else {
        // Handle non-Blob data (fallback)
        const data = JSON.parse(event.data);
        combinedHistogram.data.datasets[0].data = [data.p1];
        combinedHistogram.data.datasets[1].data = [data.p2];
        combinedHistogram.data.datasets[2].data = [data.p3];
        combinedHistogram.data.datasets[3].data = [data.p4];
        combinedHistogram.update();
    }
};
