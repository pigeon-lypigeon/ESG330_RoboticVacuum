%Blaze Witt
%ESG330 Introduction to Robotics
%Lego Mindstorm Project
clear();

%====================================================================
%base initialization
%====================================================================
% Initialize the Lego Mindstorm robot
LongLegV1 = legoev3('usb');
%Front Right Leg: D
FrontRightWheel = motor(LongLegV1, 'D');
%Front Left Leg: A
FrontLeftWheel = motor(LongLegV1, 'A');
%Gyro: 3
GyroSensor = gyroSensor(LongLegV1, 3);
%Rear Ultrasonic Sensor: 2
RearUltrasonic = sonicSensor(LongLegV1, 2);
%Front Ultrasonic Sensor: 1
FrontUltrasonic = sonicSensor(LongLegV1, 1);
%Output Order:   FrontLeftWheel, FrontRightWheel;
%Input Order = FrontUltrasonic, RearUltrasonic, GyroSensor, ColorSensor;

%====================================================================
%basic movement tech
%====================================================================

%Both forwards
function MoveForward(inputSpeed,   FrontLeftWheel, FrontRightWheel)
    %Run the front wheels forwards
    FrontLeftWheel.start(inputSpeed);
    FrontRightWheel.start(inputSpeed);
end
%Stop the car! Stop the car!
function StopTheCar(FrontLeftWheel, FrontRightWheel)
    FrontLeftWheel.stop();
    FrontRightWheel.stop();
end

function TurnLeft(inputSpeed, FrontLeftWheel, FrontRightWheel)
%Left backwards, right forwards = turn left
FrontLeftWheel.start(-inputSpeed);
FrontRightWheel.start(inputSpeed);
end

function TurnRight(inputSpeed, FrontLeftWheel, FrontRightWheel)
%The opposite of turning left; coul 
FrontLeftWheel.start(inputSpeed);
FrontRightWheel.start(-inputSpeed);
end

%====================================================================
%sensor updates
%====================================================================
function [frontDistance,rearDistance] = ReadUltrasonics(FrontUltrasonic,RearUltrasonic)
    frontDistance = readDistance(FrontUltrasonic);
    rearDistance = readDistance(RearUltrasonic);
end

%====================================================================
%90 degree turn that goes in chosen direction
%====================================================================
function TurnNinety(doTurnRight, forwardSpeed, turnSpeed,  FrontLeftWheel, FrontRightWheel, GyroSensor, RearUltrasonic)
    
    rearDistance = readDistance(RearUltrasonic);
    min=87;
    max=90;
    
    %While the change in rotation is less than 90 degrees, accounting for a
    %roughly 0.2 second stopping speed, turn left or right based on Inputs
    if(doTurnRight)
        while(readRotationAngle(GyroSensor)<min||readRotationAngle(GyroSensor)>max)
            disp(readRotationAngle(GyroSensor))
                
                if(readRotationAngle(GyroSensor)>max)
                    disp("Overturned, retrying")
                    TurnLeft(0.8*turnSpeed, FrontLeftWheel, FrontRightWheel);
                else
                    %Run the intended function
                    disp("Turning Right")
                    TurnRight(turnSpeed,  FrontLeftWheel, FrontRightWheel);
                    
                end
        end
    else
          while(readRotationAngle(GyroSensor)>-min||readRotationAngle(GyroSensor)<-max)
          

            if(rearDistance <=0.08)
                MoveForward(forwardSpeed, FrontLeftWheel, FrontRightWheel);

            elseif(readRotationAngle(GyroSensor)<-max)
                disp("Overturned, correcting")
                TurnRight(0.8*turnSpeed, FrontLeftWheel, FrontRightWheel);
            else
                disp("Turning Left")
                TurnLeft(turnSpeed, FrontLeftWheel, FrontRightWheel);
            end
            
            

          end
           %end left while loop
    end
    StopTheCar(FrontLeftWheel,FrontRightWheel);
    pause(0.05)
end


%====================================================================
%collision avoidance
%====================================================================
%A function to avoid collisions
function NoBumpy(forwardSpeed,minDistance, frontLeftWheel, frontRightWheel, frontUltrasonic, rearUltrasonic)
%A counter for how long it's been bumping
    [frontDistance,rearDistance] = ReadUltrasonics(frontUltrasonic,rearUltrasonic);
    if(frontDistance<=minDistance)
        MoveForward(-forwardSpeed, frontLeftWheel, frontRightWheel)
        pause(0.05); % Short pause to allow for movement
    elseif(rearDistance <minDistance)
        MoveForward(forwardSpeed,   frontLeftWheel, frontRightWheel);
        pause(0.05); % Short pause to allow for movement
    end
end

%move forward for a specified time while avoiding collision
function ForwardBumpy(forwardSpeed, forwardTime, minDistance, frontLeftWheel, frontRightWheel, frontUltrasonic,rearUltrasonic)
           [frontDistance,rearDistance] = ReadUltrasonics(frontUltrasonic,rearUltrasonic);
        if(forwardTime==0)
           if(frontDistance<=minDistance)
                MoveForward(-forwardSpeed, frontLeftWheel, frontRightWheel)
                pause(0.05); % Short pause to allow for movement
                forwardTime=forwardTime-0.05;
            elseif(rearDistance <=minDistance)
                MoveForward(forwardSpeed, frontLeftWheel, frontRightWheel);
                pause(0.05); % Short pause to allow for movement
                forwardTime=forwardTime-0.05;
            else
                MoveForward(forwardSpeed,frontLeftWheel,frontRightWheel);
                pause(0.05);
            end
        end
    while(forwardTime>0) 
        [frontDistance,rearDistance] = ReadUltrasonics(frontUltrasonic,rearUltrasonic);
        if(frontDistance<=minDistance)
            MoveForward(-forwardSpeed, frontLeftWheel, frontRightWheel)
            pause(0.05); % Short pause to allow for movement
            forwardTime=forwardTime-0.05;
        elseif(rearDistance <=minDistance)
            MoveForward(forwardSpeed, frontLeftWheel, frontRightWheel);
            pause(0.05); % Short pause to allow for movement
            forwardTime=forwardTime-0.05;
        else
            MoveForward(forwardSpeed,frontLeftWheel,frontRightWheel);
            pause(0.05);
            forwardTime=forwardTime-0.05;
        end
    end
    
end

%====================================================================
%core loop
%====================================================================
%initialization
doTurnRight=true;
forwardSpeed = 70;
turnSpeed = 60;
turnCounter = 0;
resetRotationAngle(GyroSensor);%setting the gyro to 0
minDistance = 0.05;%the distance at which it tries to avoid collision
targetDistance = 0.08;% the distamce at which it tries to turn
correctionWeight = 0.9;%the multiplier of the speed that it turns to correct itself
gapTime = 1.2; %time it tries to move forward between turns in seconds
while(true)
    %not running into things is key
    NoBumpy(forwardSpeed,minDistance, FrontLeftWheel, FrontRightWheel,FrontUltrasonic,RearUltrasonic);
    %update the sensors real quick
    frontDistance = ReadUltrasonics(FrontUltrasonic,RearUltrasonic);
    %drift catch statement
        if(readRotationAngle(GyroSensor)>=3)
            disp("Correcting Left")
            TurnLeft(correctionWeight*turnSpeed,  FrontLeftWheel, FrontRightWheel);
            disp(readRotationAngle(GyroSensor))
            pause(0.05);
        elseif(readRotationAngle(GyroSensor)<=-3)
            disp("Correcting Right")
            TurnRight(correctionWeight*turnSpeed,  FrontLeftWheel, FrontRightWheel);
            pause(0.05);
        end
        %if things are far enough away, go forwards
    if(frontDistance>=0.07)
        %disp("Onwards")
         MoveForward(forwardSpeed, FrontLeftWheel, FrontRightWheel)
        %if you are drifting too far to the right, turn left until not
    else
        %if you arent bumping into it and you are too close, make a turn
        %disp("all clear,onwards")

       
        
        if(frontDistance<=targetDistance)

            if(turnCounter==0)
            TurnNinety(doTurnRight, forwardSpeed, turnSpeed,  FrontLeftWheel, FrontRightWheel, GyroSensor, RearUltrasonic)
            turnCounter=turnCounter+1;
            resetRotationAngle(GyroSensor);
            ForwardBumpy(forwardSpeed, gapTime, minDistance, FrontLeftWheel, FrontRightWheel, FrontUltrasonic, RearUltrasonic)
            TurnNinety(doTurnRight, forwardSpeed, turnSpeed,  FrontLeftWheel, FrontRightWheel, GyroSensor, RearUltrasonic)
            turnCounter=turnCounter+1;
            else
            TurnNinety(doTurnRight, forwardSpeed, turnSpeed,  FrontLeftWheel, FrontRightWheel, GyroSensor, RearUltrasonic)
            turnCounter=turnCounter+1;
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

    end
end
