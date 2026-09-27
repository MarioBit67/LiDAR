# Regra 13. Protocolo do cômodo - dois giros centrais, depois cada canto mirando o oposto

Cada cômodo é capturado em estações, nesta ordem:

1. **Estação 0 - centro:** giro de 360 graus no lugar, nas faixas de inclinação que a lente exigir (no Moto
   1x: duas faixas, teto e piso). Concluída a cobertura, o app avança sozinho.
2. **Estações 1-4 - cantos:** o operador vai a cada canto em sentido horário, toca em "Cheguei" e aponta para o
   canto OPOSTO; o app guarda um leque de 60 graus em volta da mira (as duas paredes que chegam ao canto-alvo,
   com os vincos de teto e piso). O quarto canto fecha o cômodo.

Por quê: o giro central só dá rotação; os cantos dão pontos de vista separados por metros de base, então cada
canto do cômodo é visto de pelo menos duas posições - triangulação de cantos e diagonais e dimensões do cômodo
sem LiDAR, amarrando o giro central à planta. Cada frame registra estação, tipo, canto de origem e canto-alvo
(bloco LIDARCAP e registro `rtStation`).

Origem (usuário, 2026-09-26): "ir pra cada um dos cantos e capturar o canto oposto, quatro conjuntos de
capturas que te aprimoram a visão espacial do ambiente"; "após a conclusão dos dois giros iniciais, a
sequência pode então ser cada canto".

Refinamento - protocolo ADAPTATIVO (usuário, 2026-09-26): "após os dois giros iniciais... você tem material
suficiente para a planta baixa e com isso pode indicar o canto explicitamente... onde ele deve estar e para
onde ele deve apontar"; "em cômodos [em L] não serão apenas 4 capturas diagonais". Depois dos giros, a planta
estimada (mini-mapa em sprite) define as estações: cada canto convexo, mirando o canto mais distante que ele
enxerga. O giro não precisa ser no centro ("não necessariamente no centro do cômodo"): a posição relativa do
operador sai da própria planta. Sem planta confiável, volta aos 4 cantos em diagonal.

Coerência de rumo (usuário): o giroscópio deve permanecer coerente pela sessão inteira de vários cômodos, para
não haver cômodos rotacionados; os pontos de fuga reancoram entre cômodos (eixos do prédio, mod 90).

Ordem das faixas (usuário, 2026-09-26): "se começar pelas linhas do teto, você converge mais rapidamente para os
pontos de fuga, já que nem sempre o piso tem essas linhas visíveis por conta dos móveis". O giro central guia
primeiro a faixa de cima (vincos do teto), depois desce até o piso: o consenso do eixo nasce cedo e firme.
