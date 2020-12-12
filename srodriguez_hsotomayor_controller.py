"""srodriguez_hsotomayor_controller."""

#Autores
#Sebastián Rodríguez Robotham
#Herman Sotomayor

#algoritmos para mover un robot, 
#métodos
#    getXY: calcular la posición X,Y a partir del sensor de ruedas del robot.
#    hasObstacleRight: determina si hay un obstáculo del lado derecho (ajustar para cada robot, según sensores)
#    hasObstacleLeft: determina si hay un obstáculo del lado izquierdo (ajustar para cada robot, según sensores)
#    avoidObstacle: genera una estrategia para evadir los obstáculos. existen 4 comportamientos implementados:
#          - magnetismLeft: al encontrar un obstáculo, lo recorre por el lado izquierdo
#          - magnetismRight: al encontrar un obstáculo, lo recorre por el lado derecho
#          - magnetismAny: al encontrar un obstáculo, intenta recorrerlo hasta que puede soltarse para seguir el punto de destino
#          - escape: al encontrar un obstáculo, intenta separarse de el lo antes posible.
#    gotoPosition: genera una ruta para llegar desde la ubicación x,y actual (A) a la ubicación x,y deseada (B)

#para determinar las posiciones, el robot asume que parte en la ubiación x=0, y=0, por tanto
#si el robot parte en el centro del tablero, entonces podría ir a la posición 
#x=-1, y=-1 (moverze hacia el lado izquierdo inferior)


#comportamientos conocidos por mejorar:
#1. magnetismLeft tiende a separarse más de la cuenta de la caja, en proceso de revisión
#2. cuando el robot está sobre la línea X o Y del punto de destino, tiende a demorarse más
#      debido a que intenta cambiar de zona (esá al límite entre cambios de zonas)
#3. aún no tiene implementada lógica de retrocesos: a pesar que tiene un componente
#      de aleatoriedad que aplica al pasar varias veces por el mismo lugar, en ocasiones
#      se queda detenido en las puntas de las cajas, sin posibilidad de detectar que se 
#      encuentra estático (los sensores de las ruedas se siguen moviendo
#

#este código es experimental, y tiene por objetivo implementar las funciones básicas
#para movilizar al robot.
#el código no esá optimizado, y falta aplicar algunas estandarizaciones y buenas prácticas productivas.
#Si va a utilizarlo en otros proyectos, por favor hacer referencia a los autores en los métodos.

#Ultima actualización: 12 diciembre 2020, 02:00 AM

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, DistanceSensor, Motor, PositionSensor, Brake
import math

TIME_STEP = 64
MAX_SPEED = 6.28
PI = 3.1415926535897932384626433832795028841
fullWheelTurnRadians = PI * 2 #6.28318531

# create the Robot instance.
robot = Robot()


#*******************************************************************
# initialize devices
#*******************************************************************
ps = []
psNames = [
    'ps0', 'ps1', 'ps2', 'ps3',
    'ps4', 'ps5', 'ps6', 'ps7'
]

for i in range(8):
    ps.append(robot.getDistanceSensor(psNames[i]))
    ps[i].enable(TIME_STEP)

#*******************************************************************
# initialize motors
#*******************************************************************
leftMotor = robot.getMotor('left wheel motor')
rightMotor = robot.getMotor('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

#*******************************************************************
# initialize wheel sensors
#*******************************************************************
#robot.enablePositionSensor()
leftWheelSensor = robot.getPositionSensor('left wheel sensor')
rightWheelSensor = robot.getPositionSensor('right wheel sensor')
leftWheelSensor.enable( TIME_STEP)
rightWheelSensor.enable( TIME_STEP)

#*******************************************************************
# set threshold distance
#*******************************************************************
#watch https://cyberbotics.com/doc/guide/epuck?tab-language=python
#for proximity max and min values
distanceThreshold = 80.0
# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

#FOR EPUCK
degree90Radians = 2.24*2 #wheel turns to move robot in 90°
distanciaRuedaCm = 0.255

#al comenzar, suponemos que robot comienza en x=0, y=0
#robot avanza por eje Y, el eje X es perpendicular al robot
#cada cuadro mide 25cm

#sensores --> 0 no tiene obstáculo, 4096 tiene obstáculos encima


#*******************************************************************
# obstacle detection
#*******************************************************************

#returns 3 if obstacle is in front, 1 if is on right side, 0 wihtou obstacle
def hasObstacleRight(psValues):
    if psValues[0] > distanceThreshold:
        return 3
    elif psValues[1] > distanceThreshold:
        return 2
    elif psValues[2] > distanceThreshold:
        return 1

    return 0
    
#returns 3 if obstacle is in front, 1 if is on left side, 0 wihtou obstacle
def hasObstacleLeft(psValues):
    if psValues[7] > distanceThreshold:
        return 3
    elif psValues[6] > distanceThreshold:
        return 2
    elif psValues[5] > distanceThreshold:
        return 1

    return 0


#*******************************************************************
# Geometric functions
#*******************************************************************

#returns the diagonal or a rectangle from its sides
def getDiagonalFromRectangle(x, y):
    return math.sqrt( x ** 2 + y ** 2)

#returns the width and height of a rectangle from the diagonal and the sides ratio width = 3, height = 2, ratio = 2/3
def getXyFromRectangle(diagonal, relation):
    diagonalC = diagonal ** 2
    relationC = relation ** 2 + 1
    temp1 = diagonalC / relationC
    valA = math.sqrt(temp1)
    valB = valA * relation
    return (valA, valB)
    
    
#*******************************************************************
# Position Class:
# store all robot step information
#*******************************************************************

class Positions:   
    #********* previous step wheel sensor 
    leftWheelSensorAnt = 0
    rightWheelSensorAnt = 0
    
    #********* wheel sensor difference between last step an current step
    leftWheelSensorDif = 0
    rightWheelSensorDif = 0
    
    #********* wheel sensor accumulator (exclude rotations movements)
    leftWheelSensorDifAcum = 0
    rightWheelSensorDifAcum = 0
    
    #********* x,y goal coordinates --> used to calculate new position
    destinationX = 0.0
    destinationY = 0.0
    
    #********* obstacle sensors. 3 = obstacle in-front/next to you, 2 = obstacle left/right sides, 1 = obstacle back/next to you, 0 = no obstacle
    #********* values must be setting according to robot model
    obstacleLleft = 0
    obstacleRight = 0
    obstacleStatus = ""
    turnDefinition = ""
    turnDefinitionPrev = ""
    obstacleStartingAt = 0
    
    #********* area where robot is
    locationArea = "Q1"
    locationTargetArea = ""
    
    #********* x,y coordinates 
    locationX = 0 
    locationY = 0
    
    #********* x,y incremeted in the current step
    locationIncY = 0
    locationIncX = 0
    
    #********* #avg distance in radians
    movedRadians = 0
    movedRadiansAcum = 0
    movedCmInc = 0
    
    #********* robot speed (same positive values = go forward, 
    #*********              same negative values = go back,
    #*********              left > right = turn right
    #*********              right > left = turn left
    leftSpeed = 0.3
    rightSpeed= 0.3
    
    #used to set robot speed
    robotSpeed = 1.0
    
    #********* robot area positioning
    #********* between 0.0 and 0.9999 -> Q1
    #********* between 1.0 and 1.9999 -> Q2
    #********* between 2.0 and 2.9999 -> Q3
    #********* between 3.0 and 3.9999 -> Q4
    factorY = 0
    
    #********* current job status for goToPosition
    statusGoToPosition = "" # "", "FINISH", "RUNNING", "START"
    
    #********* value = true --> doing rotation
    inRotation = False
    
    #********* in order to prevent locks, in some situations the algorithm forces the robot
    #********* to follow some random direction, during some cycles
    forceTurn = ""
    forceTurnFinish = 0
    obstacleArray = []
    
    #********* number of iterations
    numCycle = 0
    lockCycles = 200
    lockCyclesMagnetism = 100
    
    #todo: random route mode
    lockMode = ""  # "", LOCK, FORWARD
    
#*******************************************************************
# get X,Y
#*******************************************************************

#get current robot position
def getXY(localPosition, leftWheelSensorAct, rightWheelSensorAct, obstacleLleft, obstacleRight):
    localPosition.numCycle += 1
    
    #save obstacle info
    localPosition.obstacleLleft = obstacleLleft
    localPosition.obstacleRight = obstacleRight
    
    #calc Y and X real incrementals (according to wheel sensors)
    localPosition.leftWheelSensorDif = leftWheelSensorAct - localPosition.leftWheelSensorAnt
    localPosition.rightWheelSensorDif = rightWheelSensorAct - localPosition.rightWheelSensorAnt
            
    #acummulators
    localPosition.leftWheelSensorDifAcum += localPosition.leftWheelSensorDif
    localPosition.rightWheelSensorDifAcum += localPosition.rightWheelSensorDif
    
    #calculating area position (Q1, Q2, Q3, Q4)
    if (localPosition.rightWheelSensorDifAcum >= localPosition.leftWheelSensorDifAcum):
        posEnPlano = (localPosition.rightWheelSensorDifAcum - localPosition.leftWheelSensorDifAcum) / degree90Radians

        #remove double flows
        while (posEnPlano > 4.0):
            posEnPlano -= 4.0
        
        if (posEnPlano == 0.0):
            #starting position
            localPosition.locationArea = "Q1"
            localPosition.factorY = 0
        elif (posEnPlano > 0.0 and posEnPlano < 1.0) or ():
            localPosition.locationArea = "Q2"
            localPosition.factorY = posEnPlano
        elif (posEnPlano >= 1.0 and posEnPlano < 2.0):
            localPosition.locationArea = "Q3"
            localPosition.factorY = 1- (2.0 - posEnPlano)
        elif (posEnPlano >= 2.0 and posEnPlano < 3.0):
            localPosition.locationArea = "Q4"
            localPosition.factorY =  (3.0 - posEnPlano )
        elif (posEnPlano >= 3.0 and posEnPlano < 4.0):
            localPosition.locationArea = "Q1"
            localPosition.factorY = (4.0 - posEnPlano)
    else:
        posEnPlano = (localPosition.leftWheelSensorDifAcum - localPosition.rightWheelSensorDifAcum) / degree90Radians
        while (posEnPlano > 4.0):
            posEnPlano -= 4.0
            
        if (posEnPlano == 0.0):
            localPosition.locationArea = "Q1"
            localPosition.factorY = 0 # 0.0 #ojo: puede que sea 1
        elif (posEnPlano > 0.0 and posEnPlano < 1.0) or ():
            localPosition.locationArea = "Q1"
            localPosition.factorY = posEnPlano
        elif (posEnPlano >= 1.0 and posEnPlano < 2.0):
            localPosition.locationArea = "Q4"
            localPosition.factorY = 1- (2.0 - posEnPlano)
        elif (posEnPlano >= 2.0 and posEnPlano < 3.0):
            localPosition.locationArea = "Q3"
            localPosition.factorY = (3.0 - posEnPlano )
        elif (posEnPlano >= 3.0 and posEnPlano < 4.0):
            localPosition.locationArea = "Q2"
            localPosition.factorY = (4.0 - posEnPlano)
    
    #print("rightWheelSensorDifAcum - leftWheelSensorDifAcum")
    #print(localPosition.rightWheelSensorDifAcum - localPosition.leftWheelSensorDifAcum)
    
    #case one: no movement (stop or rotation)
    if (localPosition.leftSpeed + localPosition.rightSpeed == 0):
        localPosition.leftWheelSensorDif = 0
        localPosition.rightWheelSensorDif = 0
        
    #avg distance in radians
    localPosition.movedRadians = (localPosition.leftWheelSensorDif + localPosition.rightWheelSensorDif) / 2
    localPosition.movedRadiansAcum += localPosition.movedRadians     
    
    if (localPosition.movedRadians > 0):
        localPosition.movedCmInc = (localPosition.movedRadians/fullWheelTurnRadians) * distanciaRuedaCm
        ajuste1 = 1- (abs(0.5 - localPosition.factorY) / 0.5)
        #ajuste: calcula x e y a partir de cuadrado
        (valA, valB) = getXyFromRectangle(localPosition.movedCmInc, ajuste1 ) #abs(leftWheelSensorDif/rightWheelSensorDif))
        
        #print("valA({0}), valB({1}, leftWheelSensorDif({2}, rightWheelSensorDif({3}), ajuste({4})".format(valA, valB,localPosition.leftWheelSensorDif,localPosition.rightWheelSensorDif, ajuste1))
        if (localPosition.locationArea == "Q1" and localPosition.factorY < 0.5):
            localPosition.locationIncX = valB
            localPosition.locationIncY = valA
        elif (localPosition.locationArea == "Q1" and localPosition.factorY >= 0.5):
            localPosition.locationIncX = valA
            localPosition.locationIncY = valB
        elif (localPosition.locationArea == "Q2" and localPosition.factorY < 0.5):
            localPosition.locationIncX = valB
            localPosition.locationIncY = valA
        elif (localPosition.locationArea == "Q2" and localPosition.factorY >= 0.5):
            localPosition.locationIncX = valA
            localPosition.locationIncY = valB
        elif (localPosition.locationArea == "Q3" and localPosition.factorY < 0.5):
            localPosition.locationIncX = valA
            localPosition.locationIncY = valB
        elif (localPosition.locationArea == "Q3" and localPosition.factorY >= 0.5):
            localPosition.locationIncX = valB
            localPosition.locationIncY = valA
        elif (localPosition.locationArea == "Q4" and localPosition.factorY < 0.5):
            localPosition.locationIncX = valA
            localPosition.locationIncY = valB
        elif (localPosition.locationArea == "Q4" and localPosition.factorY >= 0.5):
            localPosition.locationIncX = valB
            localPosition.locationIncY = valA          
    else:
        localPosition.locationIncY = 0
        localPosition.locationIncX = 0
              

    if (localPosition.locationArea == "Q2" or localPosition.locationArea == "Q3"):
        localPosition.locationIncX = localPosition.locationIncX * -1.0
    localPosition.locationX += localPosition.locationIncX
    
    if (localPosition.locationArea == "Q3" or localPosition.locationArea == "Q4"):
        localPosition.locationIncY = localPosition.locationIncY * -1.0    
    localPosition.locationY += localPosition.locationIncY
        
    #set sensors ant
    localPosition.leftWheelSensorAnt = leftWheelSensorAct
    localPosition.rightWheelSensorAnt = rightWheelSensorAct
        
    return localPosition


#*******************************************************************
# speed method (turn, rotate, go forward)
#*******************************************************************

#set speed to rotate right
def setSpeedGoForward(localPosition):
    localPosition.leftSpeed = localPosition.robotSpeed
    localPosition.rightSpeed = localPosition.robotSpeed
    return localPosition

#set speed to rotate left
def setSpeedRotateLeft(localPosition):
    localPosition.leftSpeed = localPosition.robotSpeed
    localPosition.rightSpeed = -localPosition.robotSpeed
    return localPosition

#set speed to rotate right
def setSpeedRotateRight(localPosition):
    localPosition.leftSpeed = -localPosition.robotSpeed
    localPosition.rightSpeed = localPosition.robotSpeed
    return localPosition

#set random speed
def setRandomSpeed(localPosition):
    if localPosition.numCycle % 2 == 0:
        localPosition.leftSpeed = 0.6 * localPosition.robotSpeed * 0.6
        localPosition.rightSpeed = 0.4 * localPosition.robotSpeed * 0.6
    else:
        localPosition.leftSpeed = 0.4 * localPosition.robotSpeed * 0.6
        localPosition.rightSpeed =  0.6 * localPosition.robotSpeed * 0.6
        
    return localPosition

#set turn left fast
def setTurnLeftFast(localPosition):
    localPosition.leftSpeed =  0.1 * localPosition.robotSpeed * 0.6
    localPosition.rightSpeed =  0.9 * localPosition.robotSpeed * 0.6
    return localPosition

#set turn right fast
def setTurnRightFast(localPosition):
    localPosition.leftSpeed =  0.9 * localPosition.robotSpeed * 0.6
    localPosition.rightSpeed =  0.1 * localPosition.robotSpeed * 0.6
    return localPosition

#determine where to rotate
def calcRotate(localPosition, newLocationArea):
    localPosition.leftSpeed = 0.0
    localPosition.rightSpeed = 0.0
    localPosition.inRotation = False

    #get Q "number"
    numAct= float(localPosition.locationArea[1:2])
    numDest = float(newLocationArea[1:2])

    if (localPosition.locationArea == "Q4" and newLocationArea == "Q1"):
        localPosition = setSpeedRotateRight(localPosition)
        localPosition.inRotation = True
    elif (localPosition.locationArea == "Q1" and newLocationArea == "Q4"):
        localPosition = setSpeedRotateLeft(localPosition)
        localPosition.inRotation = True
    elif (localPosition.locationArea == "Q1" and newLocationArea == "Q3"):
        localPosition = setSpeedRotateRight(localPosition)
        localPosition.inRotation = True
    elif (localPosition.locationArea == "Q3" and newLocationArea == "Q1"):
        localPosition = setSpeedRotateRight(localPosition)
        localPosition.inRotation = True
    elif (numAct > numDest):
        localPosition = setSpeedRotateLeft(localPosition)
        localPosition.inRotation = True
    elif (numAct < numDest):
        localPosition = setSpeedRotateRight(localPosition)
        localPosition.inRotation = True
    return localPosition
    
#get locationArea according to current x,y
def calcLocationArea(localPosition):
    newLocationArea = ""
    if (localPosition.destinationX >= localPosition.locationX and localPosition.destinationY >= localPosition.locationY):
        newLocationArea = "Q1"
    elif (localPosition.destinationX >= localPosition.locationX and localPosition.destinationY <= localPosition.locationY):
        newLocationArea = "Q4"
    elif (localPosition.destinationX <= localPosition.locationX and localPosition.destinationY >= localPosition.locationY):
        newLocationArea = "Q2"
    elif (localPosition.destinationX <= localPosition.locationX and localPosition.destinationY <= localPosition.locationY):
        newLocationArea = "Q3"
        
    return newLocationArea
    
#go to position x,y from current x,y
def gotoPosition(localPosition, destinationX, destinationY):
    localPosition.destinationX = destinationX
    localPosition.destinationY = destinationY
    
    #if distance difference between currentLocation and destinationLocation is small enough, success!
    if (abs(localPosition.locationX - localPosition.destinationX) < 0.002) and (abs(localPosition.locationY - localPosition.destinationY) < 0.002):
        localPosition.leftSpeed = 0.0
        localPosition.rightSpeed= 0.0
        localPosition.statusGoToPosition = "FINISH"
        return localPosition
    else:
        #continue...
        localPosition.statusGoToPosition = "RUNNING"

        #get target locationArea according to current x,y and target x,y locationArea (Q1...)        
        targetLocationArea = calcLocationArea(localPosition)
        #get rotation to targetLocationArea
        localPosition = calcRotate(localPosition, targetLocationArea)

        print("locationArea({0}) ; targetLocationArea({1}), destinationX({2}), destinationY({3})".format(localPosition.locationArea, targetLocationArea, localPosition.destinationX, localPosition.destinationY))
       
        #if no rotation (currentLocation = targetLocation), (speed == 0), go ahead
        if (localPosition.leftSpeed == 0.0 and localPosition.rightSpeed == 0.0):
            
            #diferencia de posiciones, antes del movimiento realizado
            #esto para comparar movimiento relaizado vs movimiento esperado

            #position difference, before last movement. --> to compare real movement vs expected movement.
            difX = abs(localPosition.destinationX - localPosition.locationX - localPosition.locationIncX)
            difY = abs(localPosition.destinationY - localPosition.locationY - localPosition.locationIncY)
                
            #get x and y ration
            if (difX == 0 and difY != 0):
                xAndYRation = 1
            elif (difX == 0 and difY == 0):
                xAndYRation = 0
            else:
                xAndYRation = abs(difY / difX)
            #get real x,y from Rectangle Distance
            (valA, valB) = getXyFromRectangle(localPosition.movedCmInc, xAndYRation ) 
    
            #if it stoped, move forward
            if (valA == 0 and valB == 0):
                localPosition.leftSpeed  = 0.5 * localPosition.robotSpeed
                localPosition.rightSpeed = 0.5 * localPosition.robotSpeed
            elif (localPosition.locationArea == "Q1" ):
                #assign expected x,y according to locationArea
                expectedLocationIncX = valB
                expectedLocationIncY = valA                
                #calc distance between expected position and current position
                expectedDifX = abs(expectedLocationIncX - localPosition.locationIncX)
                expectedDifY = abs(expectedLocationIncY - localPosition.locationIncY)
                #adjust wheel speed according to targetPosition distance --> 
                localPosition.leftSpeed  = localPosition.robotSpeed * (expectedDifX / (expectedDifX + expectedDifY))
                localPosition.rightSpeed = localPosition.robotSpeed * (expectedDifY / (expectedDifX + expectedDifY))
                    
            elif (localPosition.locationArea == "Q2" ):
                #assign expected x,y according to locationArea
                expectedLocationIncX = valB
                expectedLocationIncY = valA
                #calc distance between expected position and current position
                expectedDifX = abs(expectedLocationIncX + localPosition.locationIncX) #negativo
                expectedDifY = abs(expectedLocationIncY - localPosition.locationIncY)
                #adjust wheel speed according to targetPosition distance --> 
                localPosition.leftSpeed  = localPosition.robotSpeed * (expectedDifY / (expectedDifX + expectedDifY))
                localPosition.rightSpeed = localPosition.robotSpeed * (expectedDifX / (expectedDifX + expectedDifY))
            elif (localPosition.locationArea == "Q3" ):
                #assign expected x,y according to locationArea
                expectedLocationIncX = valB
                expectedLocationIncY = valA
                #calc distance between expected position and current position
                expectedDifX = abs(expectedLocationIncX + localPosition.locationIncX)
                expectedDifY = abs(expectedLocationIncY + localPosition.locationIncY)
                #adjust wheel speed according to targetPosition distance --> 
                localPosition.leftSpeed  = localPosition.robotSpeed * (expectedDifX / (expectedDifX + expectedDifY))
                localPosition.rightSpeed = localPosition.robotSpeed * (expectedDifY / (expectedDifX + expectedDifY))
            elif (localPosition.locationArea == "Q4" ):
                #assign expected x,y according to locationArea
                expectedLocationIncX = valB
                expectedLocationIncY = valA
                #calc distance between expected position and current position
                expectedDifX = abs(expectedLocationIncX - localPosition.locationIncX) #negativo
                expectedDifY = abs(expectedLocationIncY + localPosition.locationIncY)
                #adjust wheel speed according to targetPosition distance --> 
                localPosition.leftSpeed  = localPosition.robotSpeed * (expectedDifY / (expectedDifX + expectedDifY))
                localPosition.rightSpeed = localPosition.robotSpeed * (expectedDifX / (expectedDifX + expectedDifY))
                
    return localPosition


#force turn by numTimes cycles    
def setForceTurn(localPosition, numTimes, lockMode, turn):
    #numTimes == 0, unLock, otherwise: random
    if (numTimes == 0):
        localPosition.forceTurn = ""
        localPosition.forceTurnFinish = 0
        localPosition.lockMode = ""
    elif (turn == "LEFT"):
        localPosition.forceTurn = "LEFT"
        localPosition.forceTurnFinish = localPosition.numCycle + numTimes
        localPosition.lockMode = lockMode
    elif (turn == "RIGHT"):
        localPosition.forceTurn = "RIGHT"
        localPosition.forceTurnFinish = localPosition.numCycle + numTimes
        localPosition.lockMode = lockMode
    elif localPosition.numCycle % 2 == 0:
        localPosition.forceTurn = "LEFT"
        localPosition.forceTurnFinish = localPosition.numCycle + numTimes
        localPosition.lockMode = lockMode
    else:
        localPosition.forceTurn = "RIGHT"
        localPosition.forceTurnFinish = localPosition.numCycle + numTimes
        localPosition.lockMode = lockMode
            
    return localPosition


def addCycles(localPosition):
    localPosition.lockCycles += 100
    
    if (localPosition.lockCycles >= 10000):
        localPosition.lockCycles = 200
    
    return localPosition
    
def addCyclesMagnetism(localPosition):
    localPosition.lockCyclesMagnetism += 5
    
    if (localPosition.lockCyclesMagnetism >= 2000):
        localPosition.lockCyclesMagnetism = 100
    
    return localPosition
    
    
#if position was visited twice, take random route
def evaluateLocks(localPosition):
    numCyclesthreshold = 50   #if it was visited before that, discard

    #calculate threshold --> last incremental distance
    xAdd = abs(localPosition.locationIncX)
    yAdd = abs(localPosition.locationIncY)
    if (xAdd > yAdd):
        factorDif = xAdd * 2
    else:
        factorDif = yAdd * 2
        
    obstacleFound = {}
    for dataFilter in localPosition.obstacleArray:
        #if position was visited previously, take new route for 400 cycles
        if abs(dataFilter["x"] - localPosition.locationX) <= factorDif and abs(dataFilter["y"] - localPosition.locationY) <= factorDif and localPosition.numCycle > dataFilter["numCycle"] + numCyclesthreshold :
            obstacleFound = dataFilter
            localPosition = addCycles(localPosition)
            localPosition = setForceTurn(localPosition, localPosition.lockCycles, "LOCK", "")
            #print("si lo encontre, forzar giro aleatorio")
            break 
    
    if (obstacleFound == {}):
        #print("************************************** INSERTADO")
        obstacleItem = {}
        obstacleItem["x"] = localPosition.locationX
        obstacleItem["y"] = localPosition.locationY
        obstacleItem["obstacleLleft"] = localPosition.obstacleLleft
        obstacleItem["obstacleRight"] = localPosition.obstacleRight
        obstacleItem["numCycle"] = localPosition.numCycle
        localPosition.obstacleArray.append(obstacleItem)
    else:
        localPosition.obstacleArray = []
        
    return localPosition
    
#turn left or right
def setRandomTurn(localPosition):
    turn = ""
    if localPosition.numCycle % 2 == 0:
        turn = "LEFT"
    else:
        turn = "RIGHT"
        
    return turn
    
    
steps = "uno"

#initialize methods
localPosition = Positions()


def isRandomRoute(localPosition):
    return localPosition.forceTurnFinish > 0


#avoid obstacle.
#avoidMode = "magnetismAny", "magnetismLeft", "magnetismRight", "escape"
def avoidObstacle(localPosition, avoidMode):
    #if locking---
    if (isRandomRoute(localPosition) == True):
        #print("modo evita LOCK (lockMode = {2}) paso mientras {0} >= {1}".format(localPosition.forceTurnFinish,localPosition.numCycle, localPosition.lockMode ))
        if (localPosition.numCycle > localPosition.forceTurnFinish ):
            localPosition = setForceTurn(localPosition, 0, "", "")
           
    #if obstacle exists previously
    if (localPosition.obstacleStatus != ""):
        #evaluete infinite route
        if (localPosition.inRotation == False and localPosition.obstacleStartingAt + 500 >= localPosition.numCycle ):
            #si lleva muchos step, evalua cambio de estrategia
            localPosition = evaluateLocks(localPosition)
            
        #if obstacle found, try to avoid it
        if (localPosition.obstacleStatus == "STARTING"):
            localPosition.obstacleStatus = "ROTATE"
            
            #determine where to rotate
            if localPosition.forceTurn != "":
                localPosition.turnDefinition = localPosition.forceTurn
            elif (avoidMode == "magnetismLeft"):
                localPosition.turnDefinition = "LEFT"
            elif (avoidMode == "magnetismRight"):
                localPosition.turnDefinition = "RIGHT"
            elif (localPosition.obstacleLleft > 0 and localPosition.obstacleRight == 0): #obstacle is in left side
                localPosition.turnDefinition = "RIGHT"
            elif (localPosition.obstacleRight > 0 and localPosition.obstacleLleft == 0): #obstacle is in right side
                localPosition.turnDefinition = "LEFT"
            elif (avoidMode == "magnetismAny"):
                if (localPosition.turnDefinitionPrev == "RIGHT"):
                    localPosition.turnDefinition = "RIGHT"
                elif (localPosition.turnDefinitionPrev == "LEFT"):
                    localPosition.turnDefinition = "LEFT"
                else:
                    localPosition.turnDefinition = "LEFT"
                #if (localPosition.lockMode == ""):
                #    localPosition = setForceTurn(localPosition, 2000, "LOCK")
                #    localPosition.turnDefinition = localPosition.forceTurn
            elif (localPosition.obstacleRight >= 1 and localPosition.obstacleRight <= 2 and localPosition.obstacleLleft >= 1 and localPosition.obstacleLleft <= 2):
                localPosition.turnDefinition = "FORWARD"
            elif (localPosition.obstacleRight >= 3 and localPosition.obstacleLleft >= 3): #obstacle is in-front of both sides
                localPosition.turnDefinition = "RIGHT"
            else: #go wherever you want! found your route
                if (localPosition.lockMode == ""):
                    localPosition = addCycles(localPosition)
                    localPosition = setForceTurn(localPosition, localPosition.lockCycles, "LOCK", "")
                    localPosition.turnDefinition = localPosition.forceTurn
                #localPosition.turnDefinition = "LEFT" #setRandomTurn(localPosition) #"LEFT"
            
        #next, rotate acoording to last definition
        if (localPosition.obstacleStatus == "ROTATE"):
            #go to next locationArea, according to localPosition.turnDefinition
            localPosition.obstacleStatus = "POSITIONING"
            newLocationAreaNumber = float(localPosition.locationArea[1:2])
            if (localPosition.turnDefinition == "LEFT"):
                newLocationAreaNumber += 1
            elif (localPosition.turnDefinition == "RIGHT"):
                newLocationAreaNumber -= 1
            elif (localPosition.turnDefinition == "BACK"):
                newLocationAreaNumber += 2
                
            if (newLocationAreaNumber == 0):
                newLocationAreaNumber = 4
            elif (newLocationAreaNumber == -1):
                newLocationAreaNumber = 3
            elif (newLocationAreaNumber == 5):
                newLocationAreaNumber = 1
            elif (newLocationAreaNumber == 6):
                newLocationAreaNumber = 2
                
            targetArea = "Q{0}".format(int(newLocationAreaNumber))
            if (localPosition != targetArea):
                localPosition = calcRotate(localPosition, targetArea)

        #then, apply
        if (localPosition.obstacleStatus == "POSITIONING"):
            #if no obstacle, be free! (or not, it's depends -continue with the wall or not)
            if (localPosition.obstacleRight == 0 and localPosition.obstacleLleft == 0): 
                localPosition.inRotation = False
                localPosition.obstacleStatus = ""
                #recalculate targetArea according to 
                localPosition.locationTargetArea = calcLocationArea(localPosition)
                
                if (avoidMode == "magnetismRight"): #wall magnetism 
                    localPosition = setTurnRightFast(localPosition)
                elif (avoidMode == "magnetismLeft"): #wall magnetism 
                    localPosition = setTurnLeftFast(localPosition)
                
                elif (avoidMode == "escape"): #despega de muro
                    if (localPosition.turnDefinitionPrev == "LEFT"):
                        localPosition = setTurnLeftFast(localPosition)
                    elif (localPosition.turnDefinitionPrev == "RIGHT"):
                        localPosition = setTurnRightFast(localPosition)
                #next, magnetismAny      
                elif (localPosition.turnDefinitionPrev == "FORWARD" or localPosition.turnDefinitionPrev == ""):
                    localPosition.leftSpeed = 0.4
                    localPosition.rightSpeed = 0.4  
                elif (localPosition.locationTargetArea == "Q1" and localPosition.locationArea == "Q2"):
                    if (localPosition.turnDefinitionPrev == "LEFT"):
                        #mantiene pegado al muro
                        localPosition = setTurnRightFast(localPosition)
                    elif (localPosition.turnDefinitionPrev == "RIGHT"):
                         #despega de muro
                        localPosition = setTurnRightFast(localPosition)
                elif (localPosition.locationTargetArea == "Q1" and localPosition.locationArea == "Q3"):
                    if (localPosition.turnDefinitionPrev == "LEFT"):
                        #mantiene pegado al muro
                        localPosition = setTurnRightFast(localPosition)
                    elif (localPosition.turnDefinitionPrev == "RIGHT"):
                        #mantiene pegado al muro
                        localPosition = setTurnLeftFast(localPosition)
                elif (localPosition.locationTargetArea == "Q1" and localPosition.locationArea == "Q4"):
                    if (localPosition.turnDefinitionPrev == "LEFT"):
                        #mantiene pegado al muro
                        localPosition = setTurnRightFast(localPosition)
                    elif (localPosition.turnDefinitionPrev == "RIGHT"):
                        #mantiene pegado al muro
                        localPosition = setTurnLeftFast(localPosition)
                elif (localPosition.locationTargetArea == "Q2" and localPosition.locationArea == "Q3"):
                    if (localPosition.turnDefinitionPrev == "LEFT"):
                        #mantiene pegado al muro
                        localPosition = setTurnRightFast(localPosition)
                    elif (localPosition.turnDefinitionPrev == "RIGHT"):
                         #despega de muro
                        localPosition = setTurnRightFast(localPosition)
                elif (localPosition.locationTargetArea == "Q2" and localPosition.locationArea == "Q1"):
                    if (localPosition.turnDefinitionPrev == "LEFT"):
                        #despega de muro
                        localPosition = setTurnLeftFast(localPosition)
                    elif (localPosition.turnDefinitionPrev == "RIGHT"):
                        #mantiene pegado al muro
                        localPosition = setTurnLeftFast(localPosition)
                elif (localPosition.locationTargetArea == "Q2" and localPosition.locationArea == "Q4"):
                    if (localPosition.turnDefinitionPrev == "LEFT"):
                        #mantiene pegado al muro
                        localPosition = setTurnRightFast(localPosition)
                    elif (localPosition.turnDefinitionPrev == "RIGHT"):
                        #mantiene pegado al muro
                        localPosition = setTurnLeftFast(localPosition)
                elif (localPosition.locationTargetArea == "Q3" and localPosition.locationArea == "Q2"):
                    if (localPosition.turnDefinitionPrev == "LEFT"):
                         #despega de muro
                        localPosition = setTurnLeftFast(localPosition)                         
                    elif (localPosition.turnDefinitionPrev == "RIGHT"):
                         #mantiene pegado al muro
                        localPosition = setTurnLeftFast(localPosition)
                elif (localPosition.locationTargetArea == "Q3" and localPosition.locationArea == "Q4"):
                    if (localPosition.turnDefinitionPrev == "LEFT"):
                        #mantiene pegado al muro
                        localPosition = setTurnRightFast(localPosition)
                    elif (localPosition.turnDefinitionPrev == "RIGHT"):
                         #despega de muro
                        localPosition = setTurnRightFast(localPosition)
                elif (localPosition.locationTargetArea == "Q3" and localPosition.locationArea == "Q1"):
                    if (localPosition.turnDefinitionPrev == "LEFT"):
                        #mantiene pegado al muro
                        localPosition = setTurnRightFast(localPosition)
                    elif (localPosition.turnDefinitionPrev == "RIGHT"):
                        #mantiene pegado al muro
                        localPosition = setTurnLeftFast(localPosition)
                elif (localPosition.locationTargetArea == "Q4" and localPosition.locationArea == "Q3"):
                    if (localPosition.turnDefinitionPrev == "LEFT"):
                         #despega de muro
                        localPosition = setTurnLeftFast(localPosition)                         
                    elif (localPosition.turnDefinitionPrev == "RIGHT"):
                        #mantiene pegado al muro
                        localPosition = setTurnLeftFast(localPosition)
                elif (localPosition.locationTargetArea == "Q4" and localPosition.locationArea == "Q1"):
                    if (localPosition.turnDefinitionPrev == "LEFT"):
                        #mantiene pegado al muro
                        localPosition = setTurnRightFast(localPosition)
                    elif (localPosition.turnDefinitionPrev == "RIGHT"):
                         #despega de muro
                        localPosition = setTurnRightFast(localPosition)
                elif (localPosition.locationTargetArea == "Q4" and localPosition.locationArea == "Q2"):
                    if (localPosition.turnDefinitionPrev == "LEFT"):
                        #mantiene pegado al muro
                        localPosition = setTurnRightFast(localPosition)
                    elif (localPosition.turnDefinitionPrev == "RIGHT"):
                        #mantiene pegado al muro
                        localPosition = setTurnLeftFast(localPosition)
                elif (localPosition.turnDefinitionPrev == "RIGHT"):
                    #despega de muro
                    localPosition = setTurnLeftFast(localPosition)
                elif (localPosition.turnDefinitionPrev == "LEFT"):
                    #despega de muro
                    localPosition = setTurnRightFast(localPosition)
                elif (localPosition.turnDefinitionPrev == "FORWARD"):
                    localPosition.leftSpeed = 0.8
                    localPosition.rightSpeed = 0.8
                    
                if (localPosition.lockMode == "" or localPosition.lockMode == "FORWARD" or localPosition.lockMode == "MAGNETISM"):
                    if (avoidMode == "magnetismRight"):
                        localPosition = setForceTurn(localPosition, 120, "MAGNETISM", "RIGHT")
                    elif (avoidMode == "magnetismLeft"):
                        localPosition = setForceTurn(localPosition, 120, "MAGNETISM", "LEFT")
                    elif (avoidMode == "magnetismAny" and localPosition.turnDefinitionPrev == "LEFT"):
                        localPosition = setForceTurn(localPosition, 120, "MAGNETISM", "LEFT")
                    elif (avoidMode == "magnetismAny" and localPosition.turnDefinitionPrev == "RIGHT"):
                        localPosition = setForceTurn(localPosition, 120, "MAGNETISM", "RIGHT")
                    elif (avoidMode == "escape"):
                        localPosition = addCycles(localPosition)
                        localPosition = setForceTurn(localPosition, localPosition.lockCycles, "FORWARD", "")
                        localPosition.forceTurn = "" #todo: probar quitando esto
                #localPosition.forceTurnFinish = localPosition.numCycle + 100 #TODO: CAMBIAR A CM

            elif (localPosition.obstacleRight <= 2 and localPosition.obstacleLleft <= 2):
                #esquivar obstáculos
                if (localPosition.obstacleRight >=2 ):
                    localPosition = setTurnLeftFast(localPosition)
                    localPosition.inRotation = False
                elif (localPosition.obstacleRight > 0 ):
                    localPosition = setTurnLeftFast(localPosition)
                    localPosition.inRotation = False
                elif (localPosition.obstacleLleft >= 2 ):
                    localPosition = setTurnRightFast(localPosition)
                    localPosition.inRotation = False
                elif (localPosition.obstacleLleft > 0 ):
                    localPosition = setTurnRightFast(localPosition)
                    localPosition.inRotation = False
            else:
                localPosition.obstacleStatus = "STARTING"
            
        localPosition.turnDefinitionPrev = localPosition.turnDefinition
    elif (localPosition.obstacleLleft + localPosition.obstacleRight > 0 and localPosition.obstacleStatus == "" and localPosition.inRotation == False) or (localPosition.obstacleStatus != "") : # and 
        localPosition.obstacleStatus = "STARTING"
        localPosition.obstacleStartingAt = localPosition.numCycle
        #localPosition.turnDefinitionPrev = ""
        localPosition.turnDefinition = ""
        
        if (localPosition.inRotation == False and localPosition.forceTurnFinish == 0):
            #evaluate lock
            localPosition = evaluateLocks(localPosition)
            
    elif localPosition.lockMode != "":
        print("modo lock {0}".format(localPosition.lockMode))
        if (localPosition.inRotation == False and localPosition.lockMode == "LOCK"):
            #acomoda ruedas para girar y buscar nueva ruta
            localPosition = setRandomSpeed(localPosition)
            
    return localPosition



while robot.step(TIME_STEP) != -1:

    # Read the sensors:
    psValues = []
    for i in range(8):
        psValues.append(ps[i].getValue())
        
    # detect obstacles --> check for each robot
    obstacleRight = hasObstacleRight(psValues)
    obstacleLleft = hasObstacleLeft(psValues)
    
    #get current position
    localPosition = getXY(localPosition, leftWheelSensor.getValue(), rightWheelSensor.getValue(), obstacleLleft, obstacleRight)    
    localPosition.robotSpeed = 2.0
    
    #get avoid obstacles strategy
    localPosition = avoidObstacle(localPosition, "magnetismAny") #magnetismAny, magnetismLeft, magnetismRight, escape
   
        
    print("x:{0} y:{1}:".format(localPosition.locationX, localPosition.locationY))
    print("obstacleStatus({0}); obstacleLleft({1}), obstacleRight({2}), inRotation({3}), forceTurn({4})".format(localPosition.obstacleStatus, obstacleLleft, obstacleRight, localPosition.inRotation, localPosition.forceTurn))
    
    
    #no obstacle, no random route, go ahead
    if (localPosition.obstacleStatus == "" and isRandomRoute(localPosition) == False):
        #continue with plan and goals.

        if (steps == "uno"):
            localPosition = gotoPosition(localPosition, 0.25, 1.50) # -0.50,-0.50)

            if (localPosition.statusGoToPosition == "FINISH"):
                print("********************************************************************")
                print("********************************************************************")
                print("********************************************************************")
                localPosition.statusGoToPosition = "START"
                steps = "dos"
            
        elif (steps == "dos"):
            localPosition = gotoPosition(localPosition, 0.0,0.9)
            if (localPosition.statusGoToPosition == "FINISH"):
                localPosition.statusGoToPosition = "START"
                steps = "tres"
                
            
    leftMotor.setVelocity(localPosition.leftSpeed)
    rightMotor.setVelocity(localPosition.rightSpeed)
        
    

        
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

#version 0.8.6