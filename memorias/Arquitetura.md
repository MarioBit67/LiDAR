# LiDAR - Arquitetura

Estado em 2026-09-26. Atualize a cada mudança estrutural (Regra 0).

## Missão

Apps de captura de última milha (Android agora, iOS depois) para reconstruir, offline, o modelo 3D de um
imóvel para fins imobiliários. Por cômodo: o operador gira 360 graus no lugar com a câmera mais aberta; cada
frame mantido leva pose absoluta e bússola; o GPS dá a referência do ponto; a geometria é refinada pelos
vincos piso/parede/teto (iOS: LiDAR direto).

Geometria é a essência (Regra 11): GPS + bússola dão os primeiros passos; o encaixe das linhas de piso,
cantos e teto completa a geometria. O resultado é o modelo mais próximo possível do ambiente; móveis e
decoração são editados depois.

## Camadas

```
 app comum (C++20)            UI, fluxo por cômodo, decisão de frames, gravação
      |  métodos v   ^ eventos (sink)
 porta abstrata de plataforma  (modelo TMobile: layout só no comum)
      |                        |
 adaptador Android            adaptador iOS (adiado)
 NativeActivity C++ puro       ARKit via runtime ObjC a partir de .cpp
      |
 core portátil (core/)        geometria, orientação, giro, registros, log, sessão
 shared replicado (shared/)   winTypes, pool (TBlock/TAlloc), TThread/TMutex, fault, testCache, sha256
```

## Core (`core/include`, `core/src`)

| módulo | papel |
|---|---|
| `capGeom` | `TVec3`, `TMat4` (coluna-maior, câmera->mundo), `TIntrinsics`; rumo/inclinação de uma direção |
| `capOrient` | quaternion da plataforma -> câmera->mundo; `TSensorFrame`: `sfCapture` (ARKit), `sfEastNorthUp` (vetor de rotação Android), `sfNorthWestUp` (CoreMotion); rotação nativa do sensor (0/90/180/270) |
| `capSpin` | `TSpinTracker`: grade de bins de rumo por faixa de inclinação; modo círculo (giro central) ou LEQUE (canto: bins relativos à 1a mira estável); veredito por frame (`svKeep`, `svCovered`, `svTooFast`, `svOffBand`, `svBadCompass`, `svDrifted`, `svOutside`); `ForFov` (centro) e `ForCorner` (leque 60 graus); velocidade máxima = 4 px de borrão a 1/30 s |
| `capBuf` | `TByteBuf` (bloco do pool crescente, little-endian) e `TByteReader` (com checagem de limites) |
| `capRecord` | registros: `rtPose`, `rtImage`, `rtDepth`, `rtMesh`, `rtLocation`, `rtRoom`, `rtStation` (centro / canto N mirando canto oposto), `rtVanish` (medida de ponto de fuga do keyframe), `rtLayout` (planta do cômodo no referencial do próprio cômodo); buffers como VIEWS (sem cópia) |
| `capLog` | log só-anexa `LREC`; queda no meio só perde o registro final truncado |
| `capSession` | diretório da sessão: `session.json` (manifesto, escrita+rename) + `capture.lrec`; `TSessionWriter` thread-safe (`TMutex`), cômodos com altura da câmera |
| `capJPEG` | encoder JPEG baseline próprio (JFIF 4:2:0, tabelas Anexo K) direto de YUV_420_888/NV21 - keyframes sem ida a RGB, sem dependência de third-party |
| `capFrameMeta` | metadado próprio por frame embutido no JPEG (APP11 `LIDARCAP`, v1, 300 bytes fixos): camada do APARELHO (sessão, cômodo, seq, relógios, pose, bússola, GPS, lente, faixa/bin do giro) + camada do SERVIDOR (pose refinada, sala do modelo, faces vistas), reescrita in-place sem recodificar |
| `capEXIF` | EXIF APP1 padrão por keyframe (Model, Software, DateTime/Original, Orientation calculada da POSE, FocalLength, dimensões, GPS quando houver); `jpegInsertSegment` injeta sem recodificar |
| `capVanish` | pontos de fuga por keyframe: Scharr + tensor de estrutura -> planos de interpretação; vertical pela gravidade, busca 1D do rumo dos eixos do cômodo (mod 90), refinamento conjunto (eixos restritos ao plano horizontal); rumo exato do quadro, erro de inclinação e de ortogonalidade; `TAxisCheck` = consenso do cômodo (quórum 3, +-2 graus) e veredito por quadro |
| `capLayout` | planta baixa do giro central: vincos teto/piso com pé-direito assumido (2,80), altura da câmera por voto das paredes, paredes pela linha do teto confirmadas pelo piso, polígono retilíneo (retângulo, L...), verga 2,10 m reescala e mede o pé-direito, estações nos cantos convexos mirando o canto mais distante visível |

## Convenções

- Mundo: x leste, y cima, -z norte. Pose = câmera->mundo, coluna-maior; câmera x direita, y cima, olha -z.
- Tempo: `QWORD` ns. GPS: `LONG` graus*1e7, mm. Bússola: magnético, verdadeiro e precisão em graus (NaN = desconhecido).
- Android: vetor de rotação = norte magnético; declinação aplicada offline com o GPS.

## Build

- Desktop: CMake + VS18 (MSBuild), `CLToolExe` = ppCompile original; árvore única `build/`.
- Android: script que compila cada TU por ppCompile (launcher sobre o clang do NDK) + Gradle só para empacotar
  e assinar (toolchains de `third-party` com uso consentido, sem pedir confirmação).

## Protocolo de captura (Regra 13)

Por cômodo: estação 0 = giro central (faixas conforme a lente) -> avanço automático -> estações 1-4 = cantos
em sentido horário, cada um mirando o canto oposto com leque de 60 graus -> o 4o canto fecha o cômodo.
Android: foco fixo 0,5 D (2 m) quando MANUAL_SENSOR, AE travado em 30 fps, captura "pare e fotografe".
Ferramenta desktop `tools/capInspect.cpp`: extrai JPEGs e gera frames.csv (com estação/canto) e poses.csv.

## Geometria da imagem (pontos de fuga e planta)

- Rumo: sem magnetômetro, o giroscópio dá rumo relativo contínuo na sessão; os pontos de fuga dão o rumo exato mod 90 por quadro (corrigem a deriva, conferem a coerência na captura). Com bússola, rumo absoluto = eixoPF + 90k, k pela bússola (basta erro < 45 graus).
- Planta: origem no ponto do giro (não precisa ser o centro), u = eixo A do cômodo, w = eixo B; desenhada com u para cima e w à direita é vista de cima, e o sentido horário é o de caminhada. Escala: pé-direito assumido, corrigida pela verga de 2,10 m quando achada.
- Protocolo adaptativo (em implantação): giros centrais -> planta -> estações nos cantos convexos indicadas no mini-mapa (onde ficar e para onde apontar); sem planta, 4 cantos em diagonal.
