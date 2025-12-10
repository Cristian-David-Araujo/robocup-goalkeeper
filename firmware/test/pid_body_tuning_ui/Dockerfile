FROM python:3.11-slim

WORKDIR /app

# Install dependencies
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Copy application files
COPY app.py .
COPY static/ ./static/

# Create logs directory
RUN mkdir -p logs

# Expose port
EXPOSE 8082

# Set environment variables with defaults
ENV ROBOT_IP=192.168.4.1
ENV ROBOT_PORT=12345
ENV LOCAL_LISTEN_PORT=12346
ENV TELEMETRY_RATE_HZ=50

# Run the application
CMD ["python", "app.py"]
