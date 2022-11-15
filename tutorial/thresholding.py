import numpy as np
import cv2 as cv

# Revisar distintos tipos de threshold aca
# https://docs.opencv.org/3.4.1/d7/d1b/group__imgproc__misc.html#gaa9e58d2860d4afa658ef70a9b1115576

# Funcion de callback para switch
def updateThresholding(_):
    x = cv.getTrackbarPos('Umbral', 'image')
    thresholding(x)

# Funcion de callback
def thresholding(x):
    global img, switch
    s = cv.getTrackbarPos(switch, 'image')
    if s == 0:
        _, thresh = cv.threshold(img, x, 255, cv.THRESH_TOZERO)
    else:
        _, thresh = cv.threshold(img, x, 255, cv.THRESH_BINARY)
    cv.imshow('image', thresh)


# Lee una imagen
# 0: lee en escala de grises
# 1: lee en colores
img = cv.imread('images/pato.jpg', 0)

# Crea ventana de nombre 'image'
cv.namedWindow('image')

# Crea trackbars para manejar el color
cv.createTrackbar('Umbral', 'image', 0, 255, thresholding)

# Crea switch para elegir el tipo de filtro
switch = '0: Truncado\n1: Binario\n'
cv.createTrackbar(switch, 'image', 0, 1, updateThresholding)

cv.imshow('image',img)

while(True):
    # Lee teclas
    k = cv.waitKey(1)
    # Tecla 'Esc' cierra el programa
    if k == 27:
        break

# Cierra todas las ventanas abiertas
cv.destroyAllWindows()
