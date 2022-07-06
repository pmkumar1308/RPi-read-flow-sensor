import rospy


rospy.init_node("initialise_params") 

rospy.set_param('current_mass', 0)

rospy.set_param('actuating', False)

rospy.set_param('i2c_bus_inflation',1)
rospy.set_param('i2c_bus_deflation',3)

rospy.set_param('spi_adc_pressure_channel',0)

# rospy.set_param('dac_channel_inf',DAC8532.channel_A)
# rospy.set_param('dac_channel_def',DAC8532.channel_B)

rospy.set_param("KP_inf",70000)
rospy.set_param("KD_inf",400)

rospy.set_param("KP_def", 30000)
rospy.set_param("KD_def", 200)

rospy.set_param("D_inf", 40)
rospy.set_param("D_def", 40)

rospy.set_param("mass_tol", 0.01e-6)


