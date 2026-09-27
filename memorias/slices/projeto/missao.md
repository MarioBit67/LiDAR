---
name: missao
description: Mission of E:\Projetos\lidar - iOS + Android last-mile capture apps (room-by-room 360 spin with compass/GPS) for later 3D real-estate models
metadata:
  type: project
---

Started 2026-09-26. Field ("last mile") capture apps, iOS and Android, sharing one portable C++20 core; an offline pipeline later reconstructs a 3D model of a property for real-estate use.

Operator workflow (user): enter a room, set the camera as wide as possible (0.5x), spin 360 degrees in place; every kept frame is saved with its compass orientation, so each room is north-aligned and easy to fit into the building envelope later. Indoor GPS is weak but welcome as a reference of where the operator stood.

Platforms: iOS uses the LiDAR platform for exact 3D geometry (ARKit sceneDepth/mesh, gravityAndHeading). Android: "do the best possible" even without LiDAR (ultra-wide + rotation vector + GPS, ToF when present). A physical Android test phone receives test/dev builds throughout the project. iOS is DEFERRED: it depends on a Mac agent's courtesy that does not run locally, so Android is the main focus now.

Geometry refinement idea (user): use floor creases and corners (when visible) and ceiling creases (almost always visible) to model the room mathematically; trivial with LiDAR, compensates the missing depth on Android.

**Why:** the user's stated product goal. **How to apply:** capture per-frame orientation + compass, room markers, camera height, ceiling/floor crease coverage, raw depth/mesh on iOS. See regraOuro 6-9 and [[monorepoClaude]].

Resultados esperados de uma captura (usuário, 2026-09-26): uma série de frames geolocalizados (pose absoluta +
bússola + GPS por frame), um provável mapa 2D do ambiente (planta a partir das linhas de piso/teto) e a
associação de cada frame ao modelo (cada imagem sabe de onde e para onde olhou dentro do modelo).

MÉTRICA PRINCIPAL (usuário, 2026-09-27): a ÁREA DE VASSOURA do imóvel é a informação mais desejada, e quanto mais
precisa, mais robusta a solução no mercado. Critério de decisão:
- toda mudança na planta é julgada pelo efeito na área e nas medidas das salas de referência;
- referências: 202726, real 3,00 x 3,20; escritório 2,40 x 3,00, pé-direito 2,70.
Desempates previstos:
- a distância até o piso (LiDAR no iOS) separa tampos de mesa do piso;
- no Android, o vinco do teto de cada parede e o padrão de azulejos confirmam a linha do piso.

PÉ-DIREITO ÚNICO (usuário, 2026-09-27): os cômodos de um mesmo imóvel têm o mesmo pé-direito em toda a construção.
As exceções possíveis são um banheiro rebaixado e uma sala de estar com mezanino. A medida de uma porta vale para o
imóvel inteiro. Um cômodo que discorda é exceção declarada (rebaixo/mezanino) ou erro de medida.
MEZANINO (usuário): "uma dimensão extra de complexidade na captura". Fica para depois.
