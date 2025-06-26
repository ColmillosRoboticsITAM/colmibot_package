import sys, select, termios, tty


def save_terminal_settings():
    return termios.tcgetattr(sys.stdin)


def restore_terminal_settings(old_settings):
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def get_key(settings):
	tty.setraw(sys.stdin.fileno())
    # La función se queda esperando hasta que reciba un caracter desde la terminal,
    # es decir, hasta que el usario presione una de las teclas
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
    
    # Con esta función limpiamos los caracteres que estén en los buffers de
    # entrada y salida de la terminal
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key


def main():
    running_node = True

    # Antes de ejecutar el programa se debe de guardar los parámetros de la
    # terminal donde se está ejecutando este programa
    settings = save_terminal_settings()

    print("Reading from keyboard")
    print("----------------------------")
    print("Use wasd keys to move and rotate the turtle.")
    print("Use x to stop the movement.")
    print("'q' to quit")

    while(running_node == True):
        
        chr = get_key(settings)
        
        if (chr == 'q'):
            print("Terminate teleop_colmibot node")
            running_node = False
    
    # Recuperar los párametros previos y aplicarlos a la terminal
    restore_terminal_settings(settings)


if __name__ == '__main__':
    main()