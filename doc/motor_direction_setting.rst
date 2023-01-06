change contol direction
==================
path: src/servo/servo.cpp

.. code-block:: C

  Servo_::dst_x_position(const int & pos, const int8_t flag)
  {
  	...
  
      if (pos <= (AXIS_CENTER - AXIS_INTERVAL))
        duty.x += 1; // line 1
      else if (pos > (AXIS_CENTER + AXIS_INTERVAL))
        duty.x -= 1; // line 2
  
  	...
    }
  }

Change +/- operators on line 1 and line 2 to each other
