import pygame

def init():
    # Initialisiere die pygame-Bibliothek
    pygame.init()
    # Setze das Anzeigefenster auf 400x400 Pixel
    fenster = pygame.display.set_mode((400, 400))

def getKey(key):
    # Abfrage der Tastatureingabe
    # Du kannst hier die gewünschte Logik zur Erfassung der Tastatureingaben implementieren.
    # Beispiel: Verwende die pygame.key.get_pressed() Funktion, um die Zustände der Tasten abzufragen.
    keys = pygame.key.get_pressed()
    if key == "LEFT":
        return keys[pygame.K_LEFT]
    elif key == "RIGHT":
        return keys[pygame.K_RIGHT]
    elif key == "UP":
        return keys[pygame.K_UP]
    elif key == "DOWN":
        return keys[pygame.K_DOWN]
    elif key == "w":
        return keys[pygame.K_w]
    elif key == "s":
        return keys[pygame.K_s]
    elif key == "d":
        return keys[pygame.K_d]
    elif key == "a":
        return keys[pygame.K_a]
    elif key == "q":
        return keys[pygame.K_q]
    elif key == "e":
        return keys[pygame.K_e]
    elif key == "z":
        return keys[pygame.K_z]
    elif key == "f":
        return keys[pygame.K_f]
    elif key == "g":
        return keys[pygame.K_g]
    elif key == "h":
        return keys[pygame.K_h]
    elif key == "j":
        return keys[pygame.K_j]
    else:
        return False
