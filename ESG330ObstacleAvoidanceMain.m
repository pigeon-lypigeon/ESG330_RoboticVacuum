%Blaze Witt
%ESG330 Introduction to Robotics
%Lego Mindstorm Project
clear();
% Initialize the Lego Mindstorm robot
LongLegV1 = legoev3('usb');

LongLegV1.beep(0.2)
%Define all parts
%Rear left leg: B
%RearLeftWheel = motor(LongLegV1, 'B');
%Rear Right Leg: C
%RearRightWheel = motor(LongLegV1, 'C');
%Front Right Leg: D
FrontRightWheel = motor(LongLegV1, 'D');
%Front Left Leg: A
FrontLeftWheel = motor(LongLegV1, 'A');

%Front Color Sensor: 4
%RightColorSensor = colorSensor(LongLegV1, 4);
%Gyro: 3
GyroSensor = gyroSensor(LongLegV1, 3);
%Gyro Replaced with Left Color Sensor
%LeftColorSensor = colorSensor(LongLegV1, 3);
%Rear Ultrasonic Sensor: 2
RearUltrasonic = sonicSensor(LongLegV1, 2);
%Front Ultrasonic Sensor: 1
FrontUltrasonic = sonicSensor(LongLegV1, 1);
%Output Order:   FrontLeftWheel, FrontRightWheel;
%Input Order = FrontUltrasonic, RearUltrasonic, GyroSensor, ColorSensor;


%This function updates the speed variables used for simplifying the motor
%process (rear motors are backwards)
function [forwardSpeed, backwardSpeed] = UpdateSpeed(inputSpeed)
    %returns an updated forward and backward speed based on input
    forwardSpeed = inputSpeed;
    backwardSpeed = -inputSpeed;
end
%This function turns on the wheels to move forward at a given
%speed, input negative to go backwards
function MoveForward(inputSpeed,   FrontLeftWheel, FrontRightWheel)
    %Run the front wheels forwards
    FrontLeftWheel.start(inputSpeed);
    FrontRightWheel.start(inputSpeed);
end
%Stop the car! Stop the car!
function StopTheCar(  FrontLeftWheel, FrontRightWheel)
    FrontLeftWheel.stop();
    FrontRightWheel.stop();
end
%
function TurnLeft(radialDifference,inputSpeed,   FrontLeftWheel, FrontRightWheel)
[frontSpeed, rearSpeed] = UpdateSpeed(inputSpeed);

%Run the front wheels forwards
FrontLeftWheel.start(radialDifference*frontSpeed);
FrontRightWheel.start(-frontSpeed);
end

function TurnRight(radialDifference,inputSpeed,   FrontLeftWheel, FrontRightWheel)
[frontSpeed, rearSpeed] = UpdateSpeed(inputSpeed);
%Run the front wheels forwards
FrontLeftWheel.start(-frontSpeed);
FrontRightWheel.start(radialDifference*frontSpeed);
end

%Gather optimal data for rotation rate
%{
index = 20.0;
Data = [0.0,0.0,0.0]
while(index < 100)
    radialDifference = index/100;
    TurnRight(radialDifference,50,   FrontLeftWheel, FrontRightWheel);
    GyroSensor.resetRotationAngle();
  
    pause(1);
    StopTheCar(  FrontLeftWheel, FrontRightWheel);
    RotationRate = GyroSensor.readRotationAngle();
    pause(1);
    disp(index + ", "+ RotationRate+", "+radialDifference);
    index = index + 5;
end
%}

%Return integer based on whether both sensors see black (1), one sees black(2,3), or neither see black(4) 
%Deprecated
%function [rightColor] = UpdateColorSensor(LeftColorSensor, RightColorSensor)
%    rightColor = color
%end

function [frontDistance,rearDistance] = ReadUltraSonics(FrontUltrasonic,RearUltrasonic)
    frontDistance = readDistance(FrontUltrasonic);
    rearDistance = readDistance(RearUltrasonic);
end



%A function that takes a direction to turn and rotates in it
function TurnNinety(doTurnRight,radialDifference, forwardSpeed, turnSpeed,  FrontLeftWheel, FrontRightWheel, GyroSensor, FrontUltrasonic,RearUltrasonic)
%resetRotationAngle(GyroSensor);
   rearDistance = readDistance(RearUltrasonic);
    min=87;
    max=90;
    
    %While the change in rotation is less than 90 degrees, accounting for a
    %roughly 0.2 second stopping speed, turn left or right based on Inputs
    %-abs(readRotationRate(GyroSensor))*0.02
    if(doTurnRight)
        while(readRotationAngle(GyroSensor)<min||readRotationAngle(GyroSensor)>max)
            disp(readRotationAngle(GyroSensor))
  
                if(readRotationAngle(GyroSensor)>max)
                    disp("Overturned, retrying")
                    TurnLeft(radialDifference,0.8*turnSpeed,  FrontLeftWheel, FrontRightWheel);
                else
                    %Run the intended function
                    disp("Turning Right")
                    TurnRight(radialDifference,turnSpeed,  FrontLeftWheel, FrontRightWheel);
                    
                end
        end
    else
          while(readRotationAngle(GyroSensor)>-min||readRotationAngle(GyroSensor)<-max)
         %disp("Turning min degrees Current Angle: "+abs(readRotationAngle(GyroSensor)-initialAngle))

            if(rearDistance <=0.08)
                MoveForward(forwardSpeed,   FrontLeftWheel, FrontRightWheel);

            elseif(readRotationAngle(GyroSensor)<-max)
                disp("Overturned, correcting")
                TurnRight(radialDifference,0.8*turnSpeed,  FrontLeftWheel, FrontRightWheel);
            else
                disp("Turning Left")
                TurnLeft(radialDifference,turnSpeed,  FrontLeftWheel, FrontRightWheel);
            end
            
            

          end
           %end left while loop
    end
    StopTheCar(FrontLeftWheel,FrontRightWheel);
    pause(0.05)
end






%A function to avoid collisions
function NoBumpy(forwardSpeed,   FrontLeftWheel, FrontRightWheel,FrontUltrasonic,RearUltrasonic)
%A counter for how long it's been bumping
    [frontDistance,rearDistance] = ReadUltraSonics(FrontUltrasonic,RearUltrasonic);
    while frontDistance<=0.02||rearDistance<=0.02
    if(frontDistance<=0.02)
        MoveForward(-forwardSpeed, FrontLeftWheel, FrontRightWheel)
        pause(0.05); % Short pause to allow for movement
    elseif(rearDistance <=0.02)
        MoveForward(forwardSpeed,   FrontLeftWheel, FrontRightWheel);
        pause(0.05); % Short pause to allow for movement
    end
    [frontDistance,rearDistance] = ReadUltraSonics(FrontUltrasonic,RearUltrasonic);
    end
end
%Core Loop
doTurnRight=true;
radialDifference = 1;
forwardSpeed = 70;
turnSpeed = -60;
shortPause =0.5;
turnCounter = 0;
resetRotationAngle(GyroSensor);


while(true)
    %not running into things is key
    NoBumpy(forwardSpeed,   FrontLeftWheel, FrontRightWheel,FrontUltrasonic,RearUltrasonic);
    %update the sensors real quick
    frontDistance = ReadUltraSonics(FrontUltrasonic,RearUltrasonic);
    %drift catch statement
        if(readRotationAngle(GyroSensor)>=3)
            disp("Correcting Left")
            TurnLeft(radialDifference,1*turnSpeed,  FrontLeftWheel, FrontRightWheel);
            disp(readRotationAngle(GyroSensor))
            pause(0.1);
        elseif(readRotationAngle(GyroSensor)<=-3)
            disp("Correcting Right")
            TurnRight(radialDifference,1*turnSpeed,  FrontLeftWheel, FrontRightWheel);
            pause(0.1);
        end
        %if things are far enough away, go forwards
    if(frontDistance>=0.07)
        disp("Onwards")
         MoveForward(forwardSpeed, FrontLeftWheel, FrontRightWheel)
        %if you are drifting too far to the right, turn left until not
    else
        %if you arent bumping into it and you are too close, make a turn
        disp("all clear,onwards")

       
        
        if(frontDistance<=0.12)

            if(turnCounter==0)
            TurnNinety(doTurnRight,radialDifference, forwardSpeed, turnSpeed,  FrontLeftWheel, FrontRightWheel, GyroSensor, FrontUltrasonic,RearUltrasonic)
            turnCounter=turnCounter+1
             NoBumpy(forwardSpeed,   FrontLeftWheel, FrontRightWheel,FrontUltrasonic,RearUltrasonic);
            resetRotationAngle(GyroSensor);
            MoveForward(forwardSpeed, FrontLeftWheel, FrontRightWheel)
            pause(1.5)
            TurnNinety(doTurnRight,radialDifference, forwardSpeed, turnSpeed,  FrontLeftWheel, FrontRightWheel, GyroSensor, FrontUltrasonic,RearUltrasonic)
            turnCounter=turnCounter+1
            else
            TurnNinety(doTurnRight,radialDifference, forwardSpeed, turnSpeed,  FrontLeftWheel, FrontRightWheel, GyroSensor, FrontUltrasonic,RearUltrasonic)
            turnCounter=turnCounter+1
            end
          
            resetRotationAngle(GyroSensor);
        end
        
        %if you just turned right twice, next you turn left twice, to make
        %an instant ramen pattern of movement

        if(turnCounter==2)
            doTurnRight = ~doTurnRight;
            %reset turn counter
            turnCounter = 0;
        end

        %if you have turned an odd number of times, limit the distance to
        %about 1 body

        end
end
