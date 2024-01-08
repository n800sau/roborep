. pwm.cfg
echo 0 > /sys/class/pwm/${CHIP}/${PWM0}/duty_cycle
echo step 1
echo 0 > /sys/class/pwm/${CHIP}/${PWM0}/period
echo 10000 > /sys/class/pwm/${CHIP}/${PWM0}/period
echo step 2
echo 500000 > /sys/class/pwm/${CHIP}/${PWM0}/duty_cycle
