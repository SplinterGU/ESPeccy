"""

ESPeccy - Sinclair ZX Spectrum emulator for the Espressif ESP32 SoC

Copyright (c) 2024 Juan José Ponteprino [SplinterGU]
https://github.com/SplinterGU/ESPeccy

This file is part of ESPeccy.

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.

"""

import os
import subprocess
import binascii

# Configuración de archivos y nombres de variables
variables = [
    ("ESPeccy_logo", "ESPeccy-logo-cropped.png"),
    ("Layout_ZX", "Layout_ZX-ESPeccy.png"),
    ("Layout_TK", "Layout_TK-ESPeccy.png"),
    ("ZX_Kbd", "ZX_Kbd-ESPeccy.png"),
    ("PS2_Kbd", "PS2_Kbd-ESPeccy.png"),
    ("Layout_ZX81", "Layout_ZX81-ESPeccy.png"),
]

# Función para ejecutar EBF-convert.py y generar archivos .ebf8
def generate_ebf8_files(png_file):
    subprocess.run(["python", "EBF-convert.py", png_file], check=True)

# Función de compresión con el método 0xAA
def compress_data_with_aa(data):
    compressed = bytearray()
    compressed.extend(data[:8])  # Mantener los primeros 8 bytes sin cambios

    i = 8
    while i < len(data):
        byte = data[i]
        run_length = 1

        # Contar longitud de la secuencia repetida
        while i + run_length < len(data) and data[i + run_length] == byte and run_length < 255:
            run_length += 1

        if run_length >= 5 or (byte == 0xAA and run_length >= 2):
            # Codificar secuencia: AA AA xx yy
            compressed.extend([0xAA, 0xAA, run_length, byte])
            i += run_length
        else:
            # Copiar los bytes sin compresión
            compressed.append(byte)
            if byte == 0xAA and i + 1 < len(data):
                compressed.append(data[i + 1])
                i += 2
            else:
                i += 1

    return bytes(compressed)

# Función para obtener el volcado hexadecimal del archivo .ebf8 con compresión
def hex_dump(ebf8_file):
    with open(ebf8_file, "rb") as f:
        content = f.read()

    # Comprimir la data a partir del byte 8 usando el algoritmo 0xAA
    compressed_content = compress_data_with_aa(content)
    hex_data = binascii.hexlify(compressed_content).decode("utf-8")

    # Formatear la salida en bloques de 2 dígitos (bytes)
    byte_list = [f"0x{hex_data[i:i+2]}" for i in range(0, len(hex_data), 2)]

    # Insertar saltos de línea cada 12 bytes
    lines = []
    for i in range(0, len(byte_list), 12):
        line = ", ".join(byte_list[i:i + 12])
        lines.append(line)

    formatted_hex = ",\n    ".join(lines)  # Indentación opcional para alineación bonita
    return formatted_hex

# Generar el bloque de código C para cada variable
def generate_include(variable_name, png_file):
    # Generar los archivos .ebf8 para ambas versiones
    generate_ebf8_files(png_file)

    # Obtener el volcado hexadecimal para ambas versiones
    data_hex = hex_dump(png_file.replace(".png", ".ebf8"))

    # Generar el bloque de código C
    return f"""
const uint8_t {variable_name}[] = {{
    {data_hex}
}};
"""

# Eliminar todos los archivos .ebf8 y .ebf4
def delete_ebf_files():
    for root, dirs, files in os.walk("."):
        for file in files:
            if file.endswith((".ebf8", ".ebf4", ".ebf8.png", ".ebf4.png")):
                os.remove(os.path.join(root, file))

# Construir todo el include completo
def main():
    include = "/*\nESPeccy - Sinclair ZX Spectrum emulator for the Espressif ESP32 SoC\n\nCopyright (c) 2024 Juan José Ponteprino [SplinterGU]\nhttps://github.com/SplinterGU/ESPeccy\n\nThis file is part of ESPeccy.\n\nBased on previous work by:\n- Víctor Iborra [Eremus] and David Crespo [dcrespo3d] (ESPectrum)\n  https://github.com/EremusOne/ZX-ESPectrum-IDF\n- David Crespo [dcrespo3d] (ZX-ESPectrum-Wiimote)\n  https://github.com/dcrespo3d/ZX-ESPectrum-Wiimote\n- Ramón Martinez and Jorge Fuertes (ZX-ESPectrum)\n  https://github.com/rampa069/ZX-ESPectrum\n- Pete Todd (paseVGA)\n  https://github.com/retrogubbins/paseVGA\n\nThis program is free software: you can redistribute it and/or modify\nit under the terms of the GNU General Public License as published by\nthe Free Software Foundation, either version 3 of the License, or\n(at your option) any later version.\n\nThis program is distributed in the hope that it will be useful,\nbut WITHOUT ANY WARRANTY; without even the implied warranty of\nMERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the\nGNU General Public License for more details.\n\nYou should have received a copy of the GNU General Public License\nalong with this program.  If not, see <https://www.gnu.org/licenses/>.\n*/\n\n\n#ifndef __IMAGES_H\n#define __IMAGES_H\n"

    for variable_name, png_file in variables:
        include += generate_include(variable_name, png_file)

    include += "\n#endif // __IMAGES_H\n"

    # Guardar el include generado en un archivo
    with open("images.h", "w") as f:
        f.write(include)

    print("Include generado exitosamente en 'images.h'.")

    # Eliminar archivos temporales .ebf8 y .ebf4
    delete_ebf_files()
    print("Archivos .ebf8 y .ebf4 eliminados.")

if __name__ == "__main__":
    main()
