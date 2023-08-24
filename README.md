# materia-robotica-movil-rosario

Contenido de la asignatura optativa "Robótica Móvil" de grado y doctorado del Departamento de Ciencias de la Computación de la Facultad de Ciencias Exactas, Ingeniería y Agrimensura de la Universidad Nacional de Rosario, Argentina.

Los vídeos de las clases están en youtube: [https://youtube.com/playlist?list=PL6gMSgaLifncjxm4j7S2_5ky-6kC6BUZ_](https://www.youtube.com/watch?v=eyDzGr6y-S8&list=PL6gMSgaLifncjxm4j7S2_5ky-6kC6BUZ_)

La página de la materia del DCC-FCEIA-UNR es: [https://dcc.fceia.unr.edu.ar/es/lcc/521k](https://dcc.fceia.unr.edu.ar/es/lcc/521k). Todo el contenido (slides, tps, etc) está subido en el campus virtual de la UNR: [https://comunidades.campusvirtualunr.edu.ar/course/view.php?id=5797](https://comunidades.campusvirtualunr.edu.ar/course/view.php?id=5797)

# Compilación Latex

## Dependencias
Download and install sty files: pdfpc.sty
Install them in create the directory path `$HOME/texmf/tex/latex` add the .sty files. Then run the command `sudo texhash`  in `$HOME/texmf`.

## Instalar Biber biblatex (para referencias bibliográficas)

```
sudo apt-get install biber
```

## Configurar Texstudio
Usar biber biblatex para bibliography compilation.

## Configuración de Vídeos
Utilizar el comando `ffmpeg -i inputfile.gif -movflags faststart -pix_fmt yuv420p -vf "scale=trunc(iw/2)*2:trunc(ih/2)*2" output_video.mp4` para convertir un gif en vídeo mp4.

Utilizar el comando `ffmpeg -i inputfile.mp4 -vf "select=eq(n\,0)" -q:v 3 output_image.jpg` para extraer el primer frame del vídeo y utilizarlo en las slides.



