---
name: wishlist
description: Lista de desejos do produto (passos futuros anotados pelo usuário), a enriquecer com o tempo
metadata:
  type: project
---

Lista de desejos (usuário, 2026-09-27: "nosso wish list vai se enriquecendo ao longo do tempo"). Itens por ordem de
chegada; nenhum é tarefa em curso.

1. VISTA SINTÉTICA NO LUGAR DA CÂMERA (próximo passo necessário, futuro próximo): as vistas retificadas (frontais,
   niveladas pelo vinco, equalizadas), agrupadas por parede, dão 4 imagens de referência, uma por parede. Com a
   posição do operador (giroscópio + linhas do teto) e a pose, renderizar uma imagem sintética a partir dessas
   referências, que substitui a câmera coletada. Base existente: capInspect --rectify (nivelado), --walls
   (ortofoto por parede), o wireframe (projeção pose + intrínseca).
2. Filtro de móveis (imóvel cru), filme do imóvel vazio e camadas de mobiliário do simples ao luxuoso; ver
   [[missao]].
3. Planta global do imóvel: fundir os cômodos pelas portas em comum; ver [[plantaGlobal]].
4. Grade de ladrilhos por FFT: módulo, orientação, desempate do vinco do piso, mapa de oclusão, preenchimento.
5. Pares de quadros vizinhos (correlação de fase subpixel) para nivelar a faixa do piso pelos próprios móveis.
6. Espelhos (marcar e substituir o reflexo), janelas (captura especial opcional), mezanino (dimensão extra).
