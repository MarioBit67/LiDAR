---
name: plantaGlobal
description: PENDENTE (adiado pelo usuário, 2026-09-27) - fundir as plantas de cada cômodo numa planta base global do imóvel
metadata:
  type: project
---

Pedido do usuário (2026-09-27): "mais tarde cada um desses cômodos deve ser fundido aos outros numa planta base
global do imóvel" / "podemos fazer mais tarde, deixe anotado". Antes (2026-09-26): cada cômodo tem a própria
planta, e depois o usuário pode recalibrar cada um deles dentro da planta geral.

Já existe:
- rtLayout por cômodo, no referencial do cômodo: vértices u/w, axisDeg, estações.
- Giroscópio coerente durante a sessão inteira e bússola ±22°, que resolve o mod 90 dos pontos de fuga. A ROTAÇÃO
  relativa entre os cômodos já está resolvida.

Falta a TRANSLAÇÃO:
1. Casar as portas em comum: a mesma largura (60/70/80 ou 62/72/82 cm) e a mesma parede vista dos dois lados.
2. Espessura de parede típica de 0,15 m entre as faces encostadas.
3. Ajuste conjunto das paredes compartilhadas: paralelas e coplanares.
4. Editor para o operador arrastar e recalibrar cada cômodo.
5. Com o LiDAR no iOS, tudo fica em centímetros.

**Why:** entregável final do produto. O imóvel inteiro coerente, sem cômodos girados.
**How to apply:** não mexer na coerência do giroscópio entre cômodos nem no formato do rtLayout sem pensar nesta
fusão. Ver [[missao]].

Princípio (usuário, 2026-09-27): "as imagens normalizadas são a chave para o encaixe perfeito. O correto
enquadramento das linhas de teto e piso é fundamental para o sucesso dessa etapa."
- O encaixe entre os cômodos e das paredes usa as ortofotos retificadas (frontal-horizontal) e equalizadas
  (luminância eleita pelo voto do cosseno).
- Na captura, cada quadro precisa enquadrar o vinco do teto (faixa de cima) e o do piso (faixa de baixo). Sem esses
  dois vincos, a retificação e a escala perdem a âncora.
