#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Jun 26 20:56:32 2025
Robot Car with Line Following + Wall Following
Team: Fé & Lillian 

@author: Fé
Original Algorithms by Fé
Line Follower, improved from Freenove Design
Code implementation by Fé with AI assistance
"""

from ultrasonic import Ultrasonic
from motor import Ordinary_Car
from servo import Servo
from infrared import Infrared
from adc import ADC
import time

class RobotCar:
    def __init__(self):
        # Initialize component references
        self.servo = None
        self.sonic = None
        self.motor = None
        self.infrared = None
        self.adc = None
        
        # Timing control
        self.car_record_time = time.time()
        
        # Wall following state variables
        self.wall_follow_side = 'right'  # 'right' or 'left'
        self.target_wall_distance = 8.0  # cm based on a 30cm wide corridor
        self.wall_state = 'inactive'  # 'inactive', 'verifying_right', 'following_right', 'verifying_left', 'following_left', 'stopped'
        self.wall_verification_start_time = 0.0
        self.wall_verification_duration = 1.0  # seconds
        self.initial_wall_distance = 0.0
        self.wall_distance_tolerance = 5.0  # cm
        self.current_distance = 100.0  # Initialize with safe default
        
        # Motor speed settings: Adaptive Speed Control
        self.SLOW_SPEED = 800           # Absolute speed for turns
        self.MEDIUM_SPEED = 1200        # Starting speed for straight sections
        self.MAX_SPEED = 1600           # Maximum speed cap
        self.SPEED_INCREMENT = 0.10     # 10% speed increase
        self.SPEED_TIMER = 1.0          # Speed increase interval (seconds)
        
        # Speed control variables
        self.current_speed = self.MEDIUM_SPEED  # Current motor speed
        self.last_speed_increase_time = 0.0     # Timer for speed increases
        
        # Initialize all components
        if not self.start():
            print("Failed to initialize components!")
            return
    
    def start(self):
        """Initialize components"""
        try:
            if self.servo is None:
                self.servo = Servo()
        except Exception as e:
            print(f"Servo initialization failed: {e}")
            return False
            
        try:
            if self.sonic is None:
                self.sonic = Ultrasonic()
        except Exception as e:
            print(f"Ultrasonic initialization failed: {e}")
            return False
            
        try:
            if self.motor is None:
                self.motor = Ordinary_Car()
        except Exception as e:
            print(f"Motor initialization failed: {e}")
            return False
            
        try:
            if self.infrared is None:
                self.infrared = Infrared()
        except Exception as e:
            print(f"Infrared initialization failed: {e}")
            return False
            
        try:
            if self.adc is None:
                self.adc = ADC()
        except Exception as e:
            print(f"ADC initialization failed: {e}")
            return False
        
        # Set servo to center position
        try:
            self.servo.set_servo_pwm('0', 90)
            time.sleep(0.5)
        except Exception as e:
            print(f"Servo centering failed: {e}")
            
        return True

    def close(self):
        """Clean shutdown of all components"""
        # Stop motors first
        try:
            if self.motor:
                self.motor.set_motor_model(0, 0, 0, 0)
        except Exception as e:
            print(f"Error stopping motors: {e}")
        
        # Close all components safely
        components = [
            ('Ultrasonic', self.sonic),
            ('Motor', self.motor), 
            ('Infrared', self.infrared)
        ]
        
        for name, component in components:
            if component:
                try:
                    component.close()
                except Exception as e:
                    print(f"Error closing {name}: {e}")
        
        # Close ADC separately (different method name)
        if self.adc:
            try:
                self.adc.close_i2c()
            except Exception as e:
                print(f"Error closing ADC: {e}")
                
        # Clear all references
        self.servo = None
        self.sonic = None
        self.motor = None
        self.infrared = None
        self.adc = None

    def update(self):
        """Main update loop"""
        if (time.time() - self.car_record_time) > 0.1:  # 10Hz update rate
            self.car_record_time = time.time()
            
            # Get current distance reading
            try:
                distance = self.sonic.get_distance()
                if distance is not None and distance > 0:
                    self.current_distance = distance
                # If distance is None or invalid, keep previous value
            except Exception as e:
                print(f"Ultrasonic sensor error: {e}")
                # Keep previous distance value as fallback
            
            # Check for line detection
            try:
                infrared_value = self.infrared.read_all_infrared()
                
                if infrared_value != 0:  # Line detected
                    self.handle_line_following(infrared_value)
                else:  # No line detected - switch to wall following
                    self.handle_wall_following()
                    
            except Exception as e:
                print(f"Infrared sensor error: {e}")
                # Stop motors as safety measure
                self.motor.set_motor_model(0, 0, 0, 0)

    def handle_line_following(self, infrared_value):
         """Handle line following with adaptive speed control - Phase 1"""
         # Reset wall following when line is detected
         if self.wall_state != 'inactive':
             self.wall_state = 'inactive'
             try:
                 self.servo.set_servo_pwm('0', 90)  # Center servo
             except Exception as e:
                 print(f"Error centering servo: {e}")
         
         # Adaptive Speed Control Logic
         current_time = time.time()
         
         if infrared_value == 2:  # Center sensor only - straight line
             # Gradually increase speed every SPEED_TIMER interval
             if current_time - self.last_speed_increase_time >= self.SPEED_TIMER:
                 self.last_speed_increase_time = current_time
                 self.current_speed = min(self.current_speed * (1 + self.SPEED_INCREMENT), self.MAX_SPEED)
         else:
             # Any turning or multi-sensor detection - immediate slowdown
             self.current_speed = self.SLOW_SPEED
         
         # Apply line following logic with adaptive speed
         try:
             if infrared_value == 2:  # Center sensor only - go straight
                 self.motor.set_motor_model(self.current_speed, self.current_speed, 
                                          self.current_speed, self.current_speed)
             elif infrared_value == 4:  # Left sensor - turn left gently
                 self.motor.set_motor_model(-self.current_speed, -self.current_speed, 
                                          self.current_speed, self.current_speed)
             elif infrared_value == 6:  # Left + center - turn left sharply
                 self.motor.set_motor_model(-self.current_speed, -self.current_speed, 
                                          self.current_speed, self.current_speed)
             elif infrared_value == 1:  # Right sensor - turn right gently
                 self.motor.set_motor_model(self.current_speed, self.current_speed, 
                                          -self.current_speed, -self.current_speed)
             elif infrared_value == 3:  # Right + center - turn right sharply
                 self.motor.set_motor_model(self.current_speed, self.current_speed, 
                                          -self.current_speed, -self.current_speed)
             elif infrared_value == 7:  # All sensors - prioritize left turn (intersection)
                 self.motor.set_motor_model(-self.current_speed, -self.current_speed, 
                                          self.current_speed, self.current_speed)
         except Exception as e:
             print(f"Error in line following motor control: {e}")

    def handle_wall_following(self):
        """Handle wall following state machine"""
        current_time = time.time()
        
        try:
            if self.wall_state == 'inactive':
                # Start wall following - try right wall first
                self.wall_follow_side = 'right'
                self.servo.set_servo_pwm('0', 0)  # Point right (perpendicular)
                self.wall_state = 'verifying_right'
                self.wall_verification_start_time = current_time
                self.initial_wall_distance = self.current_distance
                # Move slowly forward while verifying
                self.motor.set_motor_model(self.SLOW_SPEED, self.SLOW_SPEED, 
                                         self.SLOW_SPEED, self.SLOW_SPEED)
                
            elif self.wall_state == 'verifying_right':
                # Check if right wall is consistent
                if current_time - self.wall_verification_start_time >= self.wall_verification_duration:
                    if (self.current_distance < 15 and 
                        self.is_wall_consistent(self.current_distance)):
                        self.wall_state = 'following_right'
                    else:
                        self.wall_follow_side = 'left'
                        self.servo.set_servo_pwm('0', 180)  # Point left (perpendicular)
                        self.wall_state = 'verifying_left'
                        self.wall_verification_start_time = current_time
                        self.initial_wall_distance = self.current_distance
                else:
                    # Continue moving slowly while verifying
                    self.motor.set_motor_model(self.SLOW_SPEED, self.SLOW_SPEED, 
                                             self.SLOW_SPEED, self.SLOW_SPEED)
                        
            elif self.wall_state == 'following_right':
                # Following right wall - check if wall is lost
                if self.current_distance > 20:  # Wall lost
                    self.wall_follow_side = 'left'
                    self.servo.set_servo_pwm('0', 180)  # Point left
                    self.wall_state = 'verifying_left'
                    self.wall_verification_start_time = current_time
                    self.initial_wall_distance = self.current_distance
                else:
                    self.execute_wall_following()
                    
            elif self.wall_state == 'verifying_left':
                # Check if left wall is consistent for required duration
                if current_time - self.wall_verification_start_time >= self.wall_verification_duration:
                    if (self.current_distance < 15 and 
                        self.is_wall_consistent(self.current_distance)):
                        self.wall_state = 'following_left'
                    else:
                        self.wall_state = 'stopped'
                else:
                    # Continue moving slowly while verifying
                    self.motor.set_motor_model(self.SLOW_SPEED, self.SLOW_SPEED, 
                                             self.SLOW_SPEED, self.SLOW_SPEED)
                        
            elif self.wall_state == 'following_left':
                # Following left wall - check if wall is lost
                if self.current_distance > 20:  # Wall lost
                    self.wall_state = 'stopped'
                else:
                    self.execute_wall_following()
                    
            elif self.wall_state == 'stopped':
                # No walls found - stop moving
                self.motor.set_motor_model(0, 0, 0, 0)
                
        except Exception as e:
            print(f"Error in wall following: {e}")
            # Safety stop
            self.motor.set_motor_model(0, 0, 0, 0)

    def is_wall_consistent(self, current_distance):
        """Consistent means if the distance from the wall stay within tolerance"""
        if self.initial_wall_distance <= 0:
            return False
        return abs(current_distance - self.initial_wall_distance) <= self.wall_distance_tolerance

    def execute_wall_following(self):
        try:
            # Calculate distance error from target wall
            distance_error = self.current_distance - self.target_wall_distance
            
            # Convert distance error to motor speed adjustments
            if abs(distance_error) < 3:  # Very close to target - go straight
                self.motor.set_motor_model(self.MEDIUM_SPEED, self.MEDIUM_SPEED, 
                                         self.MEDIUM_SPEED, self.MEDIUM_SPEED)
            elif abs(distance_error) < 8:  # Moderate correction needed
                if self.wall_follow_side == 'right':
                    if distance_error > 0:  # Too far from right wall, move toward it
                        self.motor.set_motor_model(self.MEDIUM_SPEED, self.MEDIUM_SPEED, 
                                                 self.SLOW_SPEED, self.SLOW_SPEED)
                    else:  # Too close to right wall, move away from it
                        self.motor.set_motor_model(self.SLOW_SPEED, self.SLOW_SPEED, 
                                                 self.MEDIUM_SPEED, self.MEDIUM_SPEED)
                else:  # Following left wall
                    if distance_error > 0:  # Too far from left wall, move toward it
                        self.motor.set_motor_model(self.SLOW_SPEED, self.SLOW_SPEED, 
                                                 self.MEDIUM_SPEED, self.MEDIUM_SPEED)
                    else:  # Too close to left wall, move away from it
                        self.motor.set_motor_model(self.MEDIUM_SPEED, self.MEDIUM_SPEED, 
                                                 self.SLOW_SPEED, self.SLOW_SPEED)
            else:  # Sharp correction needed
                if self.wall_follow_side == 'right':
                    if distance_error > 0:  # Too far from right wall, move quickly toward it
                        self.motor.set_motor_model(self.MEDIUM_SPEED, self.MEDIUM_SPEED, 
                                                 -self.SLOW_SPEED, -self.SLOW_SPEED)
                    else:  # Too close to right wall, move quickly away from it
                        self.motor.set_motor_model(-self.SLOW_SPEED, -self.SLOW_SPEED, 
                                                 self.MEDIUM_SPEED, self.MEDIUM_SPEED)
                else:  # Following left wall
                    if distance_error > 0:  # Too far from left wall, move quickly toward it
                        self.motor.set_motor_model(-self.SLOW_SPEED, -self.SLOW_SPEED, 
                                                 self.MEDIUM_SPEED, self.MEDIUM_SPEED)
                    else:  # Too close to left wall, move quickly away from it
                        self.motor.set_motor_model(self.MEDIUM_SPEED, self.MEDIUM_SPEED, 
                                                 -self.SLOW_SPEED, -self.SLOW_SPEED)
        except Exception as e:
            print(f"Error in wall following execution: {e}")
            # Safety stop
            self.motor.set_motor_model(0, 0, 0, 0)

    def get_status(self):
        """Get current status for monitoring"""
        try:
            infrared_value = self.infrared.read_all_infrared()
            # Extract individual sensor states
            L = bool((infrared_value >> 2) & 1)  # Left sensor (bit 2)
            C = bool((infrared_value >> 1) & 1)  # Center sensor (bit 1) 
            R = bool(infrared_value & 1)         # Right sensor (bit 0)
            
            status = {
                'line_sensors': {'left': L, 'center': C, 'right': R},
                'infrared_value': infrared_value,
                'ultrasonic_distance': self.current_distance,
                'wall_state': self.wall_state,
                'wall_side': self.wall_follow_side,
                'current_speed': self.current_speed,
                'max_speed': self.MAX_SPEED
            }
            return status
        except Exception as e:
            print(f"Error getting status: {e}")
            return {
                'line_sensors': {'left': False, 'center': False, 'right': False},
                'infrared_value': 0,
                'ultrasonic_distance': self.current_distance,
                'wall_state': self.wall_state,
                'wall_side': self.wall_follow_side,
                'current_speed': self.current_speed,
                'max_speed': self.MAX_SPEED
            }


def main():
    # Initialize car
    car = RobotCar()
    
    # Check if initialization was okay
    if not all([car.servo, car.sonic, car.motor, car.infrared, car.adc]):
        print("Initialization failed!")
        return
    
    try:
        last_status_time = 0
        
        while True:
            # Main update loop
            car.update()
            
            # Print status every 2 seconds
            current_time = time.time()
            if current_time - last_status_time >= 2.0:
                last_status_time = current_time
                
                try:
                    status = car.get_status()
                    line_detected = any(status['line_sensors'].values())
                    sensors_str = f"L:{status['line_sensors']['left']} C:{status['line_sensors']['center']} R:{status['line_sensors']['right']}"
                    
                    print(f"Line: {line_detected:5} | {sensors_str} | "
                          f"Wall: {status['wall_state']:15} | "
                          f"Distance: {status['ultrasonic_distance']:5.1f}cm | "
                          f"Speed: {status['current_speed']:4.0f}/{status['max_speed']}")
                except Exception as e:
                    print(f"Status error: {e}")
            
            # Small delay to prevent excessive CPU usage
            time.sleep(0.02)  # 50Hz main loop
                
    except KeyboardInterrupt:
        print("Shutdown requested by user")
    except Exception as e:
        print(f"Unexpected error: {e}")
    finally:
        car.close()
        print("Robot car stopped successfully!")

if __name__ == '__main__':
    main()