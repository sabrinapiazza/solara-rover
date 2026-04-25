def update_odometry(self, left_ticks, right_ticks):
    # Initialize on first reading
    if self.last_left_ticks == 0 and self.last_right_ticks == 0:
        self.last_left_ticks = left_ticks
        self.last_right_ticks = right_ticks
        return

    delta_left_ticks = left_ticks - self.last_left_ticks
    delta_right_ticks = right_ticks - self.last_right_ticks
    self.last_left_ticks = left_ticks
    self.last_right_ticks = right_ticks

    # Convert ticks to distance traveled (meters)
    delta_left  = (delta_left_ticks  / self.ticks_per_rev) * (2 * math.pi * self.wheel_radius)
    delta_right = (delta_right_ticks / self.ticks_per_rev) * (2 * math.pi * self.wheel_radius)

    # Dead reckoning
    delta_dist  = (delta_left + delta_right) / 2.0
    delta_theta = (delta_right - delta_left) / self.wheel_base

    self.x     += delta_dist * math.cos(self.theta)
    self.y     += delta_dist * math.sin(self.theta)
    self.theta += delta_theta

    # Publish wheel odometry
    now = self.get_clock().now()
    odom = Odometry()
    odom.header.stamp = now.to_msg()
    odom.header.frame_id = 'odom'
    odom.child_frame_id = 'base_link'
    odom.pose.pose.position.x = self.x
    odom.pose.pose.position.y = self.y
    odom.pose.pose.orientation.z = math.sin(self.theta / 2)
    odom.pose.pose.orientation.w = math.cos(self.theta / 2)

    self.odom_pub.publish(odom)