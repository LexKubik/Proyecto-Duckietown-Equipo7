import numpy as np
import cv2 as cv

# Funcion de callback
def blending(x):
    global img1, img2
    blend = cv.addWeighted(img1, 1 - x / 100.0, img2, x / 100.0, 0)
    cv.imshow('image', blend)

# Lee las imagenes
img1 = cv.imread('images/pato_no_fail.jpg', 1)
img2 = cv.imread('images/pato_fail.jpg', 1)

# Crea ventana de nombre 'image'
cv.namedWindow('image')

# Crea trackbars para manejar el color
cv.createTrackbar('Alfa', 'image', 0, 100, blending)

cv.imshow('image', img1)

while(True):
    # Lee teclas
    k = cv.waitKey(1)
    # Tecla 'Esc' cierra el programa
    if k == 27:
        break

# Cierra todas las ventanas abiertas
cv.destroyAllWindows()
