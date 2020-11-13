# Modo de compilar
Para configurar o Makefile, utilize o CMake nesta pasta, realizando o build nela mesma:`cmake -B . -S .`. Em seguida execute `make` para compilar.

Serão gerados 3 executaveis:
1. `motion` é o algoritmo que detecta o movimento. Ele identifica o plano de fundo do vídeo vindo da camera do sistema, e então consegue detectar objetos que se sobreponham a esse plano de fundo. Se você ficar parado em frente à camera por alguns segundos você vira o plano de fundo. Com esse algoritmo podemos identificar os *blobs* - que no caso são os objetos - que são armazenados como *contours*. Uma vez identificados os *blobs* podemos calcular o centros de cada um deles, utilizando o primeiro e segundo momentos. Depois de calcular os centros, mostramo-los na imagem como pequenos circulos azuis.

2. `colorsep` identifica objetos baseados em sua cor. Ele utiliza valores no padrão HSV (similar ao RGB, mas é mais intuitivo e mais fiel à maneira como nossos olhos formam imagens) - **H**ues - tonalidade/matiz, **S**aturation - saturação, e **V**alue - o quanto a cor é mais "escura". Essecialmente, limitando as faixas de H, S e V formamos um filtro para uma faixa de cores especificas. Isso é realizado modificando os valores das variáveis iLowH, iHighH, iLowS, iHighS, iLowV, e iHighV, definindo os valores superior e inferior para a matiz, saturação e brilho (HSV).

3. `hues` é um programa auxiliar para que você possa determinar os valores dos limites superior e inferior para cada uma das componentes HSV para o objeto que você deseja detectar. Arraste os sliders de forma que a diferença entre o limite superior e inferior para cada componente seja mínimo, e que você consiga ver somente o objeto que deseja (em cor branca na mascara).
