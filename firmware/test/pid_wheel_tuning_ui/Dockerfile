FROM python:3.11-slim

WORKDIR /app

# Copy requirements first for better layer caching
COPY requirements.txt .

# Install dependencies
RUN pip install --no-cache-dir -r requirements.txt

# Copy application files
COPY . .

# Expose the port
EXPOSE 8081

# Set environment variables with defaults
ENV PID_TUNING_HOST=0.0.0.0
ENV PID_TUNING_PORT=8081
ENV ROBOT_IP=192.168.1.100
ENV ROBOT_PORT=3333
ENV SAMPLING_RATE_HZ=50

# Run the application
CMD ["python", "app.py"]
