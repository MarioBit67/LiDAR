# Regra 11. Geometria é a essência do coletor

O coletor existe para entregar geometria. A ordem é: GPS + bússola dão os primeiros passos (onde o operador
estava e para onde cada frame olha, alinhado ao norte); o encaixe perfeito das linhas do piso e do teto
(vincos parede/piso, parede/teto e cantos) completa a geometria do cômodo e o encaixe no envelope do imóvel.
Toda decisão de captura (quais frames guardar, que faixas de inclinação cobrir, o que registrar por frame)
é julgada por quanto ela ajuda a fechar essa geometria - SEM sacrificar a fidelidade da imagem (abaixo).

Origem (usuário, 2026-09-26): "geometria é a essência desse coletor... GPS + bússola dão os primeiros passos,
e o perfeito encaixe das linhas do piso e teto completam essa geometria".

Resultado final de uma captura: o modelo mais próximo possível do ambiente real. Móveis e decoração são
editados depois; as linhas de piso, cantos e teto são robustas o bastante para sustentar a detecção
geométrica correta do cômodo, mesmo com o ambiente mobiliado (usuário, 2026-09-26).

O imóvel em si deve ser capturado com cautela: cores, infiltrações, quebras, trincas e demais avarias
precisam ficar fielmente registradas (JPEG de alta qualidade, sem borrão, exposição/cor documentadas).
Depois, cada móvel poderá ser editado e removido; por isso a detecção da geometria é fundamental - é ela
que permite extrapolar corretamente pisos e paredes por trás dos móveis (usuário, 2026-09-26).

Pontos de fuga (usuário, 2026-09-26): as linhas de piso, teto e cantos seguem as três direções ortogonais do
cômodo (mundo Manhattan); seus pontos de fuga por imagem, com o vertical já fixado pela gravidade, dão a
guinada exata de cada frame (corrige a deriva do giroscópio sem bússola), alinham centro/cantos/cômodos nos
eixos do prédio e validam a intrínseca. Parte embarcada (detecção/validação), refinamento no servidor.
