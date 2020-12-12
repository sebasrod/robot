
**Autores**
* Sebastián Rodríguez Robotham
* Herman Sotomayor

**Este proyecto implementa funciones esenciales para poder controlar un robot en ambiente Webot.**


## Métodos Implementados
* **getXY**: calcular la posición X,Y a partir del sensor de ruedas del robot.
* **hasObstacleRight**: determina si hay un obstáculo del lado derecho (ajustar para cada robot, según sensores)
* **hasObstacleLeft**: determina si hay un obstáculo del lado izquierdo (ajustar para cada robot, según sensores)
* **avoidObstacle**: genera una estrategia para evadir los obstáculos. existen 4 comportamientos implementados:
    - magnetismLeft: al encontrar un obstáculo, lo recorre por el lado izquierdo
    - magnetismRight: al encontrar un obstáculo, lo recorre por el lado derecho
    - magnetismAny: al encontrar un obstáculo, intenta recorrerlo hasta que puede soltarse para seguir el punto de destino
    - escape: al encontrar un obstáculo, intenta separarse de el lo antes posible.
* **gotoPosition**: genera una ruta para llegar desde la ubicación x,y actual (A) a la ubicación x,y deseada (B)

para determinar las posiciones, el robot asume que parte en la ubiación x=0, y=0, por tanto si el robot parte en el centro del tablero, entonces podría ir a la posición x=-1, y=-1 (moverze hacia el lado izquierdo inferior)


## Comportamientos conocidos por mejorar:
1. magnetismLeft tiende a separarse más de la cuenta de la caja, en proceso de revisión
2. cuando el robot está sobre la línea X o Y del punto de destino, tiende a demorarse más debido a que intenta cambiar de zona (esá al límite entre cambios de zonas)
3. aún no tiene implementada lógica de retrocesos: a pesar que tiene un componente de aleatoriedad que aplica al pasar varias veces por el mismo lugar, en ocasiones se queda detenido en las puntas de las cajas, sin posibilidad de detectar que se encuentra estático (los sensores de las ruedas se siguen moviendo.
4. En caso de colisión con los obstaculos (es raro que suceda, pero puede ocurrir), es probable que las posiciones x,y sufran pequeñas pérdidas de precisión. El algoritmo implementado que evita obstáculos en general no tiene colisiones, pero como se comentó anteriormente, en etapas de QA se detectaron casuísticas borde.

## Notas Finales.
este código es experimental, y tiene por objetivo implementar las funciones básicas para movilizar al robot. El código no esá optimizado, y falta aplicar algunas estandarizaciones y buenas prácticas productivas.

Si va a utilizarlo en otros proyectos, por favor hacer referencia a los autores en los métodos.

Ultima actualización: 12 diciembre 2020, 02:00 AM
