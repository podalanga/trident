"""PID Controller for Line Following"""

class PIDController:
    """PID controller implementation for line following control"""
    
    def __init__(self, kp: float, ki: float, kd: float):
        """
        Initialize PID controller
        
        Args:
            kp: Proportional gain
            ki: Integral gain
            kd: Derivative gain
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.previous_error = 0.0
        self.integral = 0.0
        self.max_integral = 100.0  # Anti-windup
    
    def compute(self, error: float, dt: float) -> float:
        """
        Compute PID control output - Updated based on test_firmware logic
        
        Args:
            error: Current error value
            dt: Time step in seconds
            
        Returns:
            Control output value
        """
        # Proportional term
        P = self.kp * error
        
        # Integral term with improved anti-windup (constrain to smaller range)
        self.integral += error * dt
        self.integral = max(min(self.integral, 50.0), -50.0)  # Tighter constraint
        I = self.ki * self.integral
        
        # Derivative term with zero-division protection
        D = 0.0
        if dt > 0:
            D = self.kd * (error - self.previous_error) / dt
        
        self.previous_error = error
        
        return P + I + D
    
    def reset(self):
        """Reset controller state"""
        self.previous_error = 0.0
        self.integral = 0.0


def calculate_line_position_error(sensor_values: list) -> float:
    """
    Calculate line position error from IR sensor array
    
    Args:
        sensor_values: List of 6 IR sensor readings (0-1023)
        
    Returns:
        Position error relative to center (negative = left, positive = right)
    """
    # Normalize sensor values (higher values = more black detected)
    total = sum(sensor_values)
    
    if total < 100:  # No line detected
        return 0.0
    
    # Weighted average position (0-5 index, center is 2.5)
    weighted_sum = sum(sensor_values[i] * i for i in range(6))
    line_position = weighted_sum / total
    
    # Error relative to center (between sensor 3 and 4)
    error = line_position - 2.5
    
    return error


def detect_end_square(sensor_values: list, threshold: int = 800) -> bool:
    """
    Detect if robot is on the black end square
    
    Args:
        sensor_values: List of 6 IR sensor readings (0-1023)
        threshold: Minimum value to consider sensor detecting black
        
    Returns:
        True if end square detected (5+ sensors see black)
    """
    black_count = sum(1 for v in sensor_values if v > threshold)
    return black_count >= 5
