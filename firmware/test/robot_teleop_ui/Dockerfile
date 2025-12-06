FROM python:3.11-slim

# Set working directory
WORKDIR /app

# Install dependencies
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Copy application files
COPY app.py .
COPY static/ static/

# Expose port
EXPOSE 8080

# Environment variables with defaults
ENV TELEOP_HOST=0.0.0.0
ENV TELEOP_PORT=8080
ENV MAX_LINEAR_VEL=1.0
ENV MAX_ANGULAR_VEL=2.0
ENV ROBOT_IP=192.168.1.100
ENV ROBOT_PORT=3333
ENV COMMAND_TIMEOUT=1.0
ENV UPDATE_RATE_HZ=20

# Health check
HEALTHCHECK --interval=30s --timeout=3s --start-period=5s --retries=3 \
    CMD python -c "import urllib.request; urllib.request.urlopen('http://localhost:8080/health')" || exit 1

# Run the application
CMD ["python", "app.py"]
