import ast
def txt_w(txt_file, selected_line, content_to_replace):
    # Verificar si el archivo existe
    try:
        with open(txt_file, 'r') as archivo:
            lineas = archivo.readlines()
    except FileNotFoundError:
        # Si el archivo no existe, crearlo con 30 líneas de "0"
        lineas = ['empty:0\n' for _ in range(30)]
        with open(txt_file, 'w') as archivo:
            archivo.writelines(lineas)
    
    # Asegurarse de que haya al menos 30 líneas
    while len(lineas) < 30:
        lineas.append('empty:0\n')
    
    # Asegurarse de que la línea esté en el rango válido
    if selected_line < 1 or selected_line > 30:
        raise ValueError("La línea debe estar entre 1 y 30.")
    
    # Modificar la línea seleccionada
    lineas[selected_line - 1] = content_to_replace + '\n'
    
    # Escribir las líneas de nuevo al archivo
    with open(txt_file, 'w') as archivo:
        archivo.writelines(lineas)


def parse_my_vals(value_in):
    value_in = value_in.strip()
    try:
        return ast.literal_eval(value_in) #parse to float or int ej
    except (ValueError, SyntaxError):
        return f'{value_in}'  

def txt_r(file_path, line_especification):
    #try:
    readed_line = ""
    mydict= {}
    try:
        with open(file_path, "r", encoding="utf-8") as my_file:
            readed_file_lines = my_file.readlines()  
    except FileNotFoundError:
        # Si el archivo no existe, crearlo con 30 líneas de "0"
        readed_file_lines = ['empty:0\n' for _ in range(30)]
        with open(file_path, 'w') as my_file:
            my_file.writelines(readed_file_lines)
    #if line_especification < 1 or line_especification > len(readed_file_lines): 
    #    txt_r(file_path,line_especification) 
    #    raise ValueError("El número de línea está fuera de rango.")
    try:
        readed_line = readed_file_lines[line_especification - 1].strip()  
    except IndexError:
        readed_line = "empty:0"
    #if ":" not in readed_line:
    #    raise ValueError("Formato incorrecto. Falta ':' en la línea.")
    my_key, my_values = readed_line.split(":", 1)
    content_vals = [parse_my_vals(v) for v in my_values.split(",")]  
    mydict[my_key.strip()] = content_vals
    return mydict , readed_line
    #except Exception as E:
    #    print("Error",E)

