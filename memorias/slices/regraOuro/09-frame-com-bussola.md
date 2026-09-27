# Regra 9. Cada frame guardado leva orientação absoluta (bússola) - o cômodo nasce alinhado ao norte

O operador entra no cômodo, abre a câmera o máximo possível (0.5x) e gira 360 graus no lugar. Cada frame
mantido é salvo com a pose no mundo (x leste, y cima, -z norte) e com a bússola (magnético, verdadeiro,
precisão), para que o cômodo encaixe fácil no envelope do imóvel depois. O GPS, fraco em ambiente interno, é
registrado como referência de onde o operador estava. Os vincos piso/parede, os cantos e os vincos do teto
alimentam a modelagem matemática do cômodo; a captura precisa garantir cobertura deles e registrar a altura
da câmera.

Origem (usuário, 2026-09-26).
