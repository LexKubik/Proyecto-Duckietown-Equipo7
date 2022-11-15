import numpy as np
import cv2 as cv

# Funcion de callback que no hace nada
def nothing(x):
    pass

# Crea una imagen vacía (negro)
img = np.zeros((300,512,3), np.uint8)

# Crea ventana de nombre 'image'
cv.namedWindow('image')
# Crea trackbars para manejar el color
cv.createTrackbar('R','image',0,255,nothing)
cv.createTrackbar('G','image',0,255,nothing)
cv.createTrackbar('B','image',0,255,nothing)

while(True):
    # Muestra la imagen
    cv.imshow('image',img)

    # Lee teclas
    k = cv.waitKey(1)

    # Esc cierra el programa
    if k == 27:
        break

    # Lee la posicion de cada trackbar
    r = cv.getTrackbarPos('R','image')
    g = cv.getTrackbarPos('G','image')
    b = cv.getTrackbarPos('B','image')

    # Rellena la imagen con el color correspondiente
    img[:] = [b,g,r]

# Cierra todas las ventanas abiertas
cv.destroyAllWindows()
