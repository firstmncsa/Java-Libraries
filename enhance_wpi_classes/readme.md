MecanumDrive only excepts the TalonSRX in its constructors. If the TalonSRX's where configured for ControlMode.Velocity setting the controlMode parameter to Velocity Mode will scale the -1 to 1  ySpeed, xSpeed, and zRotation to wheel velocities


Add the following to your build.gradle
repositories {

    maven { url 'https://jitpack.io' }
    
}

dependencies {

    // Use Link to get the most current Repository
    implementation '// To Do needs new link'
	
}
