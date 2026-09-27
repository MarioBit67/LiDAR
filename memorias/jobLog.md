# LiDAR - jobLog

Registro cronológico do trabalho: o que foi feito, decidido e descartado (para não repetir becos sem saída).
Entradas mais novas no fim.

## 2026-09-26

- Projeto iniciado em repositório vazio (`master`, sem commits).
- Escopo evoluiu por diretivas do usuário: driver de sensor -> app de captura para modelo 3D de imóvel -> apps
  iOS + Android de última milha -> giro 360 por cômodo com bússola + GPS; iOS com LiDAR; Android "o possível".
- DESCARTADO: primeiro rascunho do core com STL (`std::vector`, `std::optional`, `std::thread`), `double`,
  snake_case, driver UDP/LDR1 e simulador. Violava a disciplina da casa e o novo escopo; apagado sem commit.
- DESCARTADO: host Android em Kotlin fino (modelo abCamera); usuário escolheu NativeActivity C++ puro.
- DESCARTADO: shell iOS em `.mm` (ppCompile não gateia `.mm`); o plano é chamar ARKit pelo runtime ObjC a partir de `.cpp`.
- Core reescrito no estilo da casa, todo verde no `ppCheck -deep`: capGeom, capOrient, capSpin, capBuf,
  capRecord, capLog, capSession. Teste `test/capTest.cpp` (TAbTestCache) verde no ppCheck.
- `shared` replicado para `lidar/shared` (winTypes, alloc, mem, thread, fault, libDiscipline, macros,
  testCache, sha256, arg, idle + thread/fault/abNew/abPool/sha256 .cpp), cópias verdes.
- Memórias oficiais criadas em `memorias/` (regraOuro + fatias, arquitetura, jobLog), espelhadas para o pessoal.
- Build desktop configurado (CMake 4.3.1-msvc1 do VS18, gerador "Visual Studio 18 2026", `CLToolExe` = ppCompile original). Gate ativo.
- ACHADO: `TBlock::Realloc` de `shared/alloc.h` não compila em C++ (atribui LPVOID a LPSTR); nunca instanciado no monorepo. Não usar: `TByteBuf` cresce com TAlloc novo + memcpy + Drop. A cópia replicada não foi alterada.
- ACHADO: `near` é macro do windows.h; não usar como nome.
- ACHADO: o ppCheck lê `Tipo *p = NULL;` de tipo COM desconhecido como instrução; ponha linha em branco antes da próxima declaração.
- Encoder JPEG baseline próprio (capJPEG) em vez de stb (evita third-party).
- capTest VERDE via ppCompile: geometria, orientação ENU/NWU, giro (24/24 bins, veredictos), registros, sessão (cômodos, altura, cauda truncada tolerada), JPEG decodificado pelo WIC com luma PSNR 43,7 dB.
- Regra 11 registrada (geometria é a essência; modelo mais próximo do ambiente, móveis editados depois).
- Próximo: app Android (porta de plataforma + app comum + adaptador NativeActivity); pedir consentimento para NDK/JDK/Gradle/SDK de third-party.
- Aparelho de testes: Moto G9 Play (guamp), Android 11 / API 30. SEM magnetômetro (só game rotation vector) e SEM ultra-wide (principal 4,71 mm, 4016x3016, sensor 90 graus; zoom 1.0-8.0).
- DECISÃO: sem bússola, o sensor roda contínuo pela sessão inteira (todos os cômodos no mesmo referencial) e o manifesto marca `headingRef: arbitrary`; uma única rotação global é resolvida offline (envelope / GPS).
- DECISÃO: `TSpinConfig::ForFov` deriva faixas/bins do FOV real; Moto em retrato (54 x 68,5 graus) = 2 faixas em +-20,75 graus (vincos de piso e teto), 14 bins. capTest verde.
- DECISÃO: a UI é desenhada no código comum sobre um buffer de pixels do adaptador; o adaptador só rasteriza texto (Canvas via JNI) e lê sensores.
- DIRETIVA: fidelidade da imagem para avarias (cor, infiltração, trinca) + remoção posterior de móveis com extrapolação de piso/parede pela geometria. Regra 11 corrigida.
- BACKLOG: botão "Registrar avaria" (close-up marcado, com pose); gravar metadados de exposição/ISO/balanço de branco por keyframe; considerar travar AE/AWB durante o giro do cômodo para cor consistente.
- DIRETIVA: cada frame carrega metadado nosso para a correlação frame-modelo, parte calculada embarcada e parte no servidor depois. DECISÃO: bloco APP11 `LIDARCAP` de tamanho fixo (300 B) dentro do próprio JPEG, com camada do aparelho e camada do servidor (zerada na captura, reescrita in-place).
- DECISÃO: a porta lista câmeras (fatos) e o app escolhe (regra de negócio): a mais aberta entre as de resolução plena (>= metade dos pixels da melhor) - no Moto a macro 2 MP é mais aberta e seria escolhida por engano.
- ACHADO: o ppCheck trata arquivo com marcadores JNI como shim (sem literais além de 0/1; #define é o lugar das constantes; LONGLONG não é reconhecido como tipo - usar QWORD/LONG). Separação: capJNI.cpp genérico sem literais; nomes/assinaturas em TAndroid.cpp.
- ANDROID: APK completo gerado por `mobile/android/buildAndroid.sh` (todo TU via ppCompile sobre o clang do NDK r27c, API 26, glue vendorizada, alias `--defsym=android_main=androidMain`, aapt2 + aapt add + zipalign + apksigner, keystore de debug própria em `keys/`). Instalado no Moto: abre, desenha a UI comum (texto com acentos via Canvas/JNI) e pede câmera + localização. App seguiu vivo após as permissões; teste da câmera interrompido por quedas do USB/adb (não houve reboot - uptime 27 dias).
- ACHADO: `TBlock::operator->` não é const; em método const use `Pspin()`.
- PRÓXIMO: validar câmera/prévia/giro/gravação no aparelho quando o adb voltar; `adb pull /sdcard/Android/data/io.aeroblox.lidar/files/` para inspecionar a sessão.
- PRIMEIRA CAPTURA REAL (escritório do usuário, Moto G9 Play): 2 sessões; a 2a com 28/28 bins (2 faixas x 14), 4000x3000, 1-2,7 MB/JPEG, ~77 s, 7623 poses, 0 fixes de GPS (interno). Ferramenta desktop `tools/capInspect.cpp` extrai JPEGs + frames.csv/poses.csv. Vincos teto/parede, cantos, batentes e rodapé visíveis.
- PROBLEMAS vistos nas fotos: borrão de movimento (limite 27 graus/s era folgado: fx 2944 px = 51 px/grau) e foco perdido no teto liso (AF contínuo caçando). Bug: após encoder ocupado, o 1o frame passava sem checagem de velocidade.
- CORREÇÃO: "pare e fotografe" (maxRate = 4 px de borrão a 1/30 s ~ 2,4 graus/s, derivado da intrínseca), Offer em todo frame (allowKeep=false quando ocupado), foco FIXO 0,5 D (2 m) quando MANUAL_SENSOR (intrínseca constante), AE travado em 30 fps (exposição <= 1/30 s). Reinstalado.
- DIRETIVA: estações de canto (cada canto mirando o oposto) após os dois giros centrais. IMPLEMENTADO: modo leque no TSpinTracker (mira = 1o frame estável), `ForCorner`, `rtStation`, campos de estação no LIDARCAP (na cauda reservada: sessões antigas leem 0), fase do cômodo no app (centro -> andar -> canto -> ... -> fecha no 4o canto), dicas e UI por fase, capInspect com colunas de estação. capTest verde; APK reinstalado.
- DESLIZE: rodei `python3 --version` (só checagem de versão) - violação da Regra 1. Não repetir.
- Usuário capturou mais um cômodo com o protocolo de cantos (sessões 174424 e 175043 no aparelho, íntegras lá). TRANSFERÊNCIA FALHOU: USB cai a cada poucos segundos; adb pull e pull em blocos (`build/pullChunks.sh`) vieram parciais/corrompidos. Aparelho sem Wi-Fi (só dados), adb tcpip inviável. Pedido ao usuário: trocar cabo/porta ou copiar via Explorer.
- ACHADO: `pullChunks.sh` anexa o bloco mesmo após esgotar as tentativas (gerou session.json maior que o original) - corrigir antes de reusar.
- GPS: nenhum fix interno; app só lê getLastKnownLocation (sem callback de satélites no host C++ puro). Orientação ao operador: abrir a sessão perto da janela/fora para semear o fix.
- DIRETIVA: imagens sem EXIF e aparentemente rotacionadas. DECISÃO: pixels continuam na orientação nativa do sensor (a intrínseca vale para ela) + EXIF com Orientation calculada da pose (onde a gravidade cai na imagem: retrato Android = 6). App grava APP1 em cada keyframe; capInspect injeta EXIF nas capturas antigas a partir do LIDARCAP + session.json (lido com TJSON replicado de shared). Validado lendo as tags com System.Drawing: orientation 6, modelo, data, 4,71 mm. focalMm adicionado ao LIDARCAP (cauda reservada).
- USB do aparelho: não passa de ~6 MB por conexão; sessões grandes vêm pelo Explorer.

- DIRETIVA: `third-party\` do monorepo Claude = uso consentido sem confirmação ("uso consentido sem confirmação"). Corrigida a contradição entre a Regra 4 e `slices/referencia/monorepoClaude.md` (e a linha de build da Arquitetura).

## 2026-09-26 (tarde) - pontos de fuga, planta baixa, protocolo adaptativo

- PONTOS DE FUGA (`core/capVanish`): luma reduzida a <= 800 px, gradiente Scharr, NMS, tensor de estrutura 5x5 (coerência >= 0,85) -> normal do plano de interpretação por aresta. Vertical pela gravidade; busca 1D do rumo dos eixos do cômodo (mod 90); refinamento conjunto competitivo (vertical livre, eixos A/B restritos ao plano horizontal - uma linha por imagem basta). Saída: rumo do eixo do cômodo, erro de inclinação (vs gravidade) e de ortogonalidade (intrínseca). Opcional: raio + rótulo por aresta (alimenta a planta).
- DESCARTADO/APRENDIDO no detector: Sobel 3x3 dava viés de ~3 graus (anisotropia) -> Scharr; render sintético sem antialiasing enganava (real é suavizado pela ótica) -> teste supersampla 4x4; refinamento sequencial A roubava arestas de B -> atribuição competitiva conjunta; mínimo-quadrados 3D degenera com uma linha só (moldura isolada) -> eixos horizontais restritos ao plano; `small` é macro do Windows (rpcndr.h) - não usar como nome.
- CONFERÊNCIA NA CAPTURA (`TAxisCheck`): referência = CONSENSO do cômodo (maior grupo concordante em +-2 graus, quórum 3), não o 1o quadro (o 1o real estava 1,5 grau fora). Veredito por quadro: sem linhas / pendente / referência / alinhado / desalinhado. App mostra "Linhas: N alinhados, M fora, K sem linhas (desvio)". Registro `rtVanish` + resumo na cauda do LIDARCAP (roomAxisDeg, axisDevDeg, tilt/ortho em centigraus, flags, veredito; 296/300 bytes).
- DADOS REAIS: 173630 consenso 27,85 graus, alinhados com desvio ~1 grau; 175043 (quarto) consenso ~38 graus. Discrepantes a 45 graus = PISO ASSENTADO EM DIAGONAL (confirmado nas fotos). Inclinação medida ~1,5-2 graus sistemática (desalinhamento câmera/IMU ou gravidade) - servidor estima como extrínseca constante. Distorção da lente: Moto reporta zeros.
- PLANTA BAIXA (`core/capLayout`): arestas horizontais do giro central viram candidatos a vinco; teto: distância = (H-h)q, piso: h*q'. Altura da câmera por VOTO DAS PAREDES (vinco do teto x candidatos do piso, cada lado vota uma vez) - o histograma global era enganado por juntas do piso alinhadas às paredes e pelo rodapé. Paredes pela linha do teto, confirmadas pelo piso numa janela de +-6% da distância (rodapé). Raios a < 10 graus da própria linha descartados (mal condicionados). Polígono retilíneo andando as paredes em sentido horário; paralelas vizinhas ganham conector. Estações = cantos convexos, cada um mirando o canto mais distante que enxerga. Sintético: retângulo 4 cantos/alvos opostos; L 6 cantos/5 estações, 5,00 x 4,05 m (real 5 x 4).
- VERGA 2,10 m (diretiva do usuário: "a porta é mais efetiva que o pé-direito"): linhas de parede abaixo da moldura, histograma de altura; 2,10/verga reescala a planta e o pé-direito vira MEDIDA. Sintético: assumido 2,80, medido 2,58 (real 2,60).
- PLANTA REAL ainda não confiável: 175043 saiu 3,90 x 8,92 (um lado espúrio); 173630/173241 com cantos reflexos espúrios. Próxima captura com o app novo valida.
- Registro `rtLayout` (planta por cômodo, no referencial do próprio cômodo) - base da edição posterior da planta geral do imóvel.
- TRANSFERÊNCIA: `adb exec-out dd` mistura o stderr do dd do aparelho no fluxo (~77 bytes) -> redirecionar no aparelho (`"dd ... 2>/dev/null"`); `pullChunks.sh` refeito (blocos de 1 MB por arquivo, retoma, monta só se completo). MELHOR: usuário ligou Wi-Fi -> `adb tcpip 5555` + `adb connect 192.168.15.22:5555` + `ANDROID_SERIAL`: 118 MB em 23 s. Git Bash converte `/sdcard/...` -> usar `MSYS_NO_PATHCONV=1`.
- DIRETIVAS registradas (usuário): protocolo adaptativo - após os giros centrais a planta indica o canto explicitamente no mapa (sprite), onde ficar e para onde apontar; cômodo em L NÃO são 4 diagonais; o operador pode girar fora do centro (a origem da planta é o ponto do giro); cada cômodo tem sua planta e depois o usuário reposiciona cada um na planta geral; preservar o giroscópio coerente na sessão inteira (vários cômodos) para não haver cômodos rotacionados; com bússola de +-22 graus os pontos de fuga resolvem o empate mod 90 (limite teórico +-45); magnetômetro/LiDAR tornam tudo trivial, a geometria da imagem é a rede de segurança.
- DECISÕES pendentes de implementar: não parar sensores no OnPause com imóvel aberto; reancorar rumo entre cômodos pelos eixos (mesmos eixos do prédio mod 90: diferença = deriva; após reinício do sensor, 1o eixo restaura); rumo absoluto = eixoPF + 90k com k pela bússola.
- BACKLOG (usuário): marcar ESPELHOS onde o operador aparece (servidor substitui por reflexo sintético sem ele); JANELAS: detector + captura especial opcional por janela (aberta, do centro; mesma pose com o peito na parede; direita, esquerda, céu, piso), dispensáveis; PORTA como régua: largura 60/70/80 ou fora do padrão 62/72/82 (construtoras), em cm no iOS com LiDAR; Hough de segmentos longos para vincos (sugestão do usuário).

- DIRETIVA (usuário): teto primeiro no giro central - vincos do teto raramente escondidos por móveis; o consenso do eixo se forma cedo. IMPLEMENTADO em hintText (faixa mais alta incompleta primeiro).

- 3o CÔMODO (191624, 40 img, 235 fixes de GPS de rede +-105 m, 40 rtVanish do aparelho): teto primeiro funcionou (quórum no 4o quadro). ACHADO: DERIVA do giroscópio medida pelos pontos de fuga, ~5 graus em 262 s (eixo 27,5 -> 22,5). DECISÃO: (a) `TRoomLayout::AddFrame` desfaz a deriva quadro a quadro (gira as arestas para o eixo âncora; saltos > 8 graus - piso diagonal - ficam de fora); planta resolve no `AnchorDeg()`; (b) `TAxisCheck` com consenso em janela deslizante (10 quadros) que acompanha a deriva lenta, contagens no momento do veredito, `DriftDeg()`.
- Planta real ainda falha: paredes fracas espúrias (147/76 votos) aceitas pelo limiar absoluto. PRÓXIMO: vinco do teto como o SEGMENTO LONGO mais alto de cada imagem da faixa de cima (Hough de segmentos, sugestão do usuário), em vez de voto pixel a pixel. `TLayoutPlan` agora expõe as paredes candidatas (diagnóstico) e `failure` (enum `pf*`).
- DIRETIVA (usuário): cada imagem só verde após a verificação de coerência. IMPLEMENTADO: cor por veredito contra o consenso ATUAL (verde alinhado, laranja fora, cinza sem linhas, verde-claro medindo/aguardando quórum, vazio quase transparente). Sem bloquear a captura (refotografar dá o mesmo resultado).
- DIRETIVA (usuário, esboço ASCII): cobertura em PERSPECTIVA em vez de anéis - painéis por bin, faixa de cima em leque para o teto, de baixo para o piso, cantos (eixo + 45 + 90k) desenhados como aresta vertical com vincos "\ /" e "/ \"; e "evidenciando qual região você quer capturar": próximo bin vazio da faixa guiada com contorno amarelo, seta na borda quando fora da vista; retângulo amarelo = o que a câmera vê. `canvasLine`, `canvasFillQuad` novos. Verificado por screencap via adb (tocar "Iniciar cômodo" cria sessão de teste no aparelho - apagada: imovel_20260926_192954).
- Sessões no PC: todas as 6 extraídas em `build/sessions/<sessão>/` (191530 = início abandonado, 5 img).

- PLANTA REVISADA (usuário: "a planta baixa é totalmente definida pelas linhas do teto"; "use apenas as linhas do teto"; "duas ou mais linhas pode ser viga ou armários, desempate pela mais alta"). NOVO ALGORITMO: Hough 1D por quadro (pico da elevação perpendicular por lado e plano = uma linha física); vinco = a linha MAIS ALTA com >= 15% do comprimento da maior do quadro; só quadros da faixa do teto (inclinação >= 5 graus) dão vincos; paredes = agrupamento dos vincos entre quadros; piso só para a altura da câmera (voto por parede sobre linhas). DESCARTADO: voto global pixel a pixel (misturava vinco, verga, topo de móvel, viga, juntas do piso); dente pelo piso (o teto dobra junto com a viga).
- ACHADOS: (1) cadeia de deriva derrubada por um quadro espúrio (eixo +6,7 graus aceito no limite de 8) -> quadros bons da parede da porta rejeitados; agora trilha suave (0,5*salto) e aceita só +-3 graus; (2) gravidade do Moto erra 1,5-4 graus vs a vertical da imagem (montagem/fusão) -> trava de 4 graus cortava quadros bons e distorcia distâncias (~10% a 25 graus de elevação); `TTiltBias` = média das correções medidas (quadros com >= 200 arestas verticais) vira `upBias` da detecção e gira as poses na planta (a vertical de UM quadro é ruidosa demais); (3) verga "achada" no escritório era a laje/alizar (1,94 m) -> reescala pela verga DESLIGADA (medida só registrada) até reconhecer porta por verga + dois batentes; (4) quadros 19/20 do escritório = mancha de sol da janela (bordas de luz, não geometria).
- VERDADE DO ESCRITÓRIO 173630 (usuário): 2,40 (janela <-> oposta) x 3,00 (porta <-> oposta), pé-direito 2,70, cavidade para armário embutido ao lado da porta sob uma laje. RESULTADO: com 2,70 -> 2,50 x 2,86 (+4% / -5%); com 2,80 assumido -> 2,69 x 3,08. Quarto 175043: 4,00 x 3,72 (antes 3,96 x 9,04); 3o cômodo 191624: 3,03 x 2,87. Altura da câmera 1,64-1,72 m (alta; a conferir).
- FERRAMENTA: `capInspect ... --edges` gera edges_NNN.bmp (Sobel + arestas aceitas: vermelho vertical, verde A, ciano B; em pé).

- RECAPTURA DO ESCRITÓRIO (200900, centro + 4 cantos, 40 img). Deriva ~9 graus em 200 s, em degraus entre faixas.
- ALTURA DA CÂMERA PELOS CANTOS (usuário: "um único frame de canto que te permita ver os vincos do teto e piso fecham a equação"; "o teto te dá a linha vertical invisível... a linha de piso devidamente estendida encontra essa vertical"). IMPLEMENTADO `layoutPairHeight`: num mesmo quadro, os dois vincos do teto fixam o azimute do canto (q_c1/q_c0), os dois do piso têm de cruzar a MESMA vertical (q_f1/q_f0 igual, 3%); k = q_f/q_c, h = H/(1 + k); quadros das estações de canto entram só para a altura (não para paredes). DESCARTADO: arestas verticais soltas por azimute (misturam batente, janela, pé de mesa, mancha de sol; pé do canto escondido por móvel). Recaptura: h = 1,64 m por 3 cantos.
- TRAVA DE DERIVA: +-3 graus com trilha suave rejeitava 21 de 40 quadros (deriva em degraus de 3-5 graus); agora mediana dos 5 últimos eixos aceitos, +-5 graus (35 de 40 entram).
- ESTADO: recaptura 2,53 x 2,25 (real 2,40 x 3,00 - um lado 25% curto): parede w- (a da janela) sustentada por 1 quadro; nos outros quadros o vinco dela aparece no Sobel mas não é aceito (branco nas imagens de borda). PRÓXIMO: por que a direção B é rejeitada nesses quadros.

- DESCARTADO: refazer busca/refino em torno da vertical MEDIDA do quadro quando ela diverge da gravidade - no quadro 13 da recaptura a vertical medida estava errada (6,8 graus, poucas verticais agrupadas: mal condicionada) e o eixo B caiu de 1803 para 13 arestas. A gravidade (com a calibração média) é mais confiável que a vertical de um quadro. O vinco da janela rejeitado continua em aberto.
- Usuário vai recapturar os outros dois cômodos (menos móveis): "o excesso de móveis atrapalha muito".

- PRIMEIRO BOM RESULTADO REAL: 202726 (usuário: 3,00 x 3,20; pé-direito do apartamento 2,70) -> 2,94 x 3,09 (-2% / -3,4%), 4 paredes pelo teto, altura 1,54 m por 6 cantos, sem ajuste. Resíduo sistemático para menos (moldura: o vinco é a borda de baixo dela? pé-direito real ~2,76?). 203334 ainda falha (2,77 x 1,82, degrau espúrio, lado sem parede).

- DIRETIVA (usuário): "o desenho geral já parece muito bom e depois naturalmente podemos refinar" - precisão absoluta fica para depois; seguir para a planta no app (mini-mapa + cantos planejados).
- REFINAMENTOS ABERTOS da planta: (a) moldura na periferia da imagem vista pelo Sobel mas não rotulada (203334 quadros 0/1/3; 200900 quadro 13 - a janela) - tolerância de rótulo 2x NÃO resolveu (descartado); suspeita: distorção da lente -> calibrar k1 pelo fio de prumo com os vincos longos; (b) topo de guarda-roupa embutido aceito como vinco quando a moldura acima falha -> degrau falso de 24 cm (203334); (c) verga (reescala) ainda desligada.

- PLANTA NO APP (APK 20:44, instalado; ainda NÃO testado em captura): worker alimenta TRoomLayout (centro = paredes, cantos = altura) com as arestas do detector; ao sair do giro central solvePlan (no worker após o keyframe que fechou o giro, ou direto se livre) -> estações = cantos convexos do plano (fallback 4 diagonais), rtLayout gravado, EndRoom com a altura da câmera. Mini-mapa em sprite (drawPlan) GIRADO pelo rumo (o que o operador encara fica para cima): polígono, ponto do giro, estação atual em amarelo com seta tracejada até o canto-alvo, feitas em verde, medidas em pt-BR; grande no andar até o canto, pequeno no canto a canto. Dicas: "Vá ao canto amarelo do mapa (N de M)", "Aponte para onde a seta do mapa indica".

- RETIFICAÇÃO (usuário: "versão frontal horizontal de cada frame"; "cada frame tem rotação real dos três eixos de acordo com os giroscópios"): capInspect --rectify = rotação pura (K R K^-1) para a câmera virtual frontal à parede, nivelada, com descentramento (lente shift); quadro mirando canto (> 25 graus) ganha uma vista por parede; rótulo pelo rumo NO MUNDO da normal (paredeK, a mesma parede física em todos os quadros). 202726: parede do guarda-roupa frontal em 9 quadros de pontos/alturas diferentes. ACHADO: direção A não medida vinha NaN quando só B existia e antes do quórum -> derivar A de B / usar a previsão do quadro.
- COMPOSIÇÃO DAS PAREDES (usuário: "a composição de cada parede sobrepondo as diversas capturas"): capInspect --walls = ORTOFOTO métrica por parede (4 mm/px), cada pixel = ponto 3D no retângulo da parede (planta + pé-direito), projetado em cada quadro do giro central com a rotação corrigida pelos pontos de fuga do quadro (vertical real; eixo -> eixo da planta + 90k), mistura com peso concentrado no centro da imagem. ACHADOS: (1) o celular gira em torno do CORPO, não da câmera: raio ~0,30 m modelado (0 e 0,45 dão puxadores duplicados); (2) quadros com eixo > 12 graus do previsto pelo giroscópio (piso diagonal) projetavam a parede errada (porta fantasma) -> descartados; 20 de 28 quadros usados. PENDENTE: preencher buracos com os descartados (só giroscópio, prioridade baixa); estações de canto (paralaxe, exigem posição da estação); exposição variável entre quadros.

- DIRETIVA (usuário): "os quatro cantos do teto são sua referência básica de geometria; por extensão, as linhas do piso mesmo não visíveis te dão o desenho de cada parede". IMPLEMENTADO capInspect --overlay: caixa do cômodo (4 cantos do teto + 4 do piso a H abaixo, 12 arestas) projetada em cada quadro do giro com a rotação dos pontos de fuga e o raio do giro; 202726: teto sobre a moldura e cantos sobre as arestas reais na maioria; piso desenhado atrás de cama/malas. Desvios de poucos pixels restantes.
- PRÓXIMO (usuário: "janelas, portas e móveis podem e devem ser utilizados para refinamento de subpixel"): casar detalhes entre quadros vizinhos sobrepostos (correlação subpixel), resolver as correções de rotação de todos os quadros juntos com a caixa do teto como âncora, recompor as paredes.

- REFINAMENTO SUBPIXEL (tools/capMosaic.cpp, capInspect --walls --refine N): cada quadro do giro gera seu recorte da ortofoto de cada parede; casado contra o mosaico dos OUTROS (leave-one-out) por NCC de bordas, grosso (+-16 cm) -> fino (+-3 px) -> parábola; deslocamento na parede vira rotação do quadro (beta = -ds/D no vertical, alpha = dz/D no eixo direito da parede). ACHADOS: ganho 0,8 oscilava (pares de 2 quadros se cruzavam) -> 0,5; sugestão do usuário (Sobel das linhas separadoras): bordas separadas por direção, |gx| mede o deslocamento lateral e |gy| o vertical; casamentos no limite da busca descartados. 202726: desalinhamento médio 68 mm -> ~12 mm em 8 rodadas, MAS só 14 de ~30 pares entram (limiar de correlação) e os não corrigidos geram fantasmas novos (puxadores triplicados); metade de baixo com paralaxe de móveis. PRÓXIMO: ajuste conjunto de todos os quadros com peso pela qualidade e a caixa do teto como âncora; casar só na faixa de parede livre (do vinco do teto para baixo).

- AJUSTE CONJUNTO ANCORADO NO TETO (capMosaic mosaicJoint, capInspect --joint N): pares de quadros sobrepostos + âncora do vinco no topo da parede + prior fraco, mínimos quadrados de todos juntos. RESULTADO PIOR em 202726 (moldura empurrada para fora, janela duplicada, guarda-roupa partido). CAUSAS: (1) a âncora força o vinco na altura da PLANTA aproximada (h, distâncias, raio estimados) e briga com os pares; (2) a detecção do vinco no recorte pegava a borda de BAIXO da moldura (12-15 cm, mais contrastada) -> corrigido para a linha forte mais alta, mas ainda com discrepantes; (3) só 8 pares passam. DECISÃO: --joint desligado por padrão (experimental); --refine (padrão 6 rodadas quadro contra os outros) segue como padrão. PRÓXIMO de verdade: ajuste de feixe com rotações E geometria (distância de cada parede, altura da câmera, raio do giro) juntas, o teto como restrição e não alvo fixo.

- AJUSTE DE FEIXE (capMosaic mosaicBundle, capInspect --walls [--bundle N, padrão 3]): incógnitas = rotação de cada quadro, deslocamento de cada parede, e = teto acima da câmera, r = raio do giro; observações medidas por rodada e fixas: pares de pontos de imagem (blocos de 50 cm das sobreposições alinhados por bordas) e pontos do vinco do teto; resíduos com raio saindo do centro REAL do quadro; Gauss-Newton + Levenberg + Huber, Jacobiano numérico. ACHADO: resíduos em METROS na parede degeneram a escala (sala encolheu para 0,8 m, câmera subiu ao teto) -> pares reprojetados em PIXELS no outro quadro e teto como ÂNGULO (invariantes à escala); escala só pelo pé-direito e pelos priors. 202726: 2,94 x 3,09 -> 3,06 x 3,25 (real 3,00 x 3,20: +2,0% / +1,7%, antes -2/-3,4), câmera 1,72 m, raio 0,28 m, estável em 3 rodadas; paredes: guarda-roupa sem puxadores duplicados, janela limpa. Só ~20 pares/rodada (limiar de casamento). PENDENTE: mais observações (blocos menores, limiar adaptativo), buracos, estações de canto no feixe (paralaxe forte), exposição.

- ESTAÇÕES DE CANTO NO AJUSTE DE FEIXE (usuário: "imagens de cantos opostos podem gerar o preenchimento desses buracos"; "por ser um canto, essas imagens te dão perspectivas ortogonais daqueles cantos"): +2 incógnitas (x, z) por estação; início = canto da planta mais oposto à mira, 25% para dentro; 1a rodada com busca larga; rodadas quadro-contra-outros só com o centro. 202726: pares 20 -> ~120-138 por rodada; planta 2,995 x 3,186 (real 3,00 x 3,20: -0,2% / -0,4%) após 5 rodadas (padrão --bundle 5); câmera 1,81 m, raio 0,32 m. Composição: cantos com peso 1e-4 onde o centro vê (só preenchem buracos) - sem buracos, mas com costuras onde objetos fora do plano (armário, janela recuada, móveis) têm paralaxe entre pontos de vista. PRÓXIMO: costura ótima (transição em pouca textura); objetos fora do plano como camada (servidor).

- EQUALIZAÇÃO DE LUMINÂNCIA (usuário: "quanto menor o ângulo de ataque, mais fiel à luminância da parede"; "as paredes têm luminância constante, média ponderada pelo cosseno do ângulo de ataque"; "um único fator de luminância, eleito pelo voto balanceado pelo cosseno"). IMPLEMENTADO mosaicElect: em cada ponto de uma grade da parede, cor eleita = média das vistas pesada pelo cosseno do ângulo de ataque; fator de cada quadro por parede = MEDIANA das razões eleito/próprio nos pontos compartilhados (mesmo conteúdo dos dois lados: móvel escuro não distorce); composição também pesada pelo cosseno; saída limitada a 255. DESCARTADOS: ganhos por mínimos quadrados em log (ok, mas substituídos pelo voto); mediana do NÍVEL de cada quadro (quadros do piso viam móveis -> ganho alto demais, estourava). 202726: paredes 0/1/2 de tom uniforme, costuras de brilho quase sumidas. Vinheta da lente não modelada ainda.

## ESTADO PARA RETOMAR (fim da tarde 2026-09-26)

- Repositório sem commits. Novos: `core/capVanish`, `core/capLayout`, registros `rtVanish` (8) e `rtLayout` (9), contadores vanish/layouts no manifesto, `canvasLine`, `build/norm.sh` (renormaliza CRLF - o sed do Git Bash tira CR), `build/pullChunks.sh` corrigido.
- capTest VERDE (pontos de fuga: eixo +-0,2 grau, deriva de 3 graus detectada, consenso; planta retângulo/L/verga). APK com pontos de fuga + conferência INSTALADO no Moto (planta ainda NÃO está no app).
- EM CURSO: integração da planta no app (`mobile/app/capApp.cpp`): membros Playout/Pplan/Pedges/PcornerStep/PcornerCount/PplanWanted e cCeilingM já declarados; FALTA: AddFrame das arestas do giro central no EncodePending, solvePlan ao sair do centro, estações pelo plano (fallback 4 cantos), drawPlan (sprite: polígono, ponto do giro, canto atual amarelo + seta de mira, feitos verdes, medidas "4,45 x 3,22 m"), dicas por fase, WriteLayout, EndRoom com altura da câmera. Enum TPlanFailure precisa do prefixo `pf` (ppCheck enumprefix) - capLayout.h ainda não passa no ppCheck.
- Sessões: todas no PC. 174424 e 175043 em `build/sessions/pull/`; extraídas em `build/sessions/<sessão>/` (JPEG com EXIF, frames.csv com colunas de ponto de fuga, plan_room0.svg); brutas em `build/sessions/files/` e `build/sessions/pull/`.
- Ferramenta: `capInspect <sessão> <saída> [--vanish] [--ceiling m] [--mingrad n]`.

## 2026-09-27 - DISPLAY DOS CANTOS + CANTO DEDUZIDO PELA DIAGONAL
- Os três círculos (drawFan) saíram. A captura de canto usa a mesma vista em perspectiva do giro central (drawCoverage genérico via TSpinTracker::BinCenterDeg/BinWidthDeg; uma faixa só ocupa a altura toda).
- Planta em sprite no rpCorner, centralizada entre a barra do topo e a perspectiva: onde ficar (amarelo) e para onde apontar (tracejado + anel).
- Não há canto #1 fixo. Enquanto o leque não tem bin, o app escolhe, entre as estações pendentes, aquela cuja diagonal (estação -> alvo na planta, girada para o heading do giroscópio) fica mais próxima da mira. A sugerida leva 15 graus de vantagem, para desempatar diagonais vizinhas no L (5 cantos de 90 graus + 1 saliente; as estações são os 5). Sem planta, a diagonal do eixo VP + 45 + 90k nomeia o canto k. Estado: PcornerMask (estações feitas), PcornerSlot (a da vez), PcornerStep (contagem).
- A arquitetura é agnóstica: o polígono vem dos vincos do teto alinhados pelos pontos de fuga, com qualquer número de vértices.
- Verde = eixo VP do quadro a até ±2 graus do consenso do cômodo; laranja = fora (piso diagonal, mancha de sol, linhas de móvel, salto do giroscópio). O quadro é salvo mesmo assim.
- APK instalado via Wi-Fi. Falta validar em campo.

## 2026-09-27 - REQUISITO: PLANTA GLOBAL DO IMÓVEL (backlog)
- Mais tarde, cada cômodo será fundido aos outros numa planta base global do imóvel.
- Já existe: rtLayout por cômodo no referencial do próprio cômodo (vértices u/w, axisDeg). O giroscópio é coerente durante a sessão inteira, e a bússola (±22°) resolve o mod 90 dos pontos de fuga. Com isso, a ROTAÇÃO relativa entre os cômodos já está resolvida.
- Falta a TRANSLAÇÃO. Plano:
  1. Casar as portas em comum: a mesma largura (60/70/80 ou 62/72/82) e a mesma parede de lados opostos.
  2. Espessura de parede típica de 0,15 m entre as faces encostadas.
  3. Ajuste conjunto de todas as paredes compartilhadas (paralelas e coplanares).
  4. Editor para o operador arrastar e recalibrar cada cômodo.

## 2026-09-27 - VIOLAÇÃO DA REGRA 1 (registro)
- Usei um script Python temporário (scratchpad, já apagado) para aplicar edições em capApp.cpp. Isso viola a
  Regra 1. O código do projeto não foi afetado.
- Daqui em diante, as edições são feitas só com a ferramenta Edit ou com sed/bash.
- A planta global ficou adiada pelo usuário e anotada em slices/projeto/plantaGlobal.md.

## 2026-09-27 - PRINCÍPIO: IMAGEM NORMALIZADA + VINCOS ENQUADRADOS
- Usuário: as imagens normalizadas (retificadas e equalizadas) são a chave para o encaixe perfeito, e o
  enquadramento correto das linhas de teto e piso é fundamental. Anotado em slices/projeto/plantaGlobal.md.
- Consequência para a captura: o verde de cada bin continua exigindo a coerência pelos pontos de fuga. Candidato
  futuro: exigir também o vinco visível (teto na faixa de cima, piso na de baixo).

## 2026-09-27 - PRINCÍPIO: O MESMO CANTO EM VÁRIOS ENQUADRAMENTOS
- Usuário: nas 40 imagens, cada canto aparece mais de uma vez, sob enquadramentos diferentes. A correlação entre
  essas vistas tem valor intrínseco.
- Hoje:
  - O bundle correlaciona blocos de parede entre pares de quadros e usa o vinco do TETO por quadro.
  - A altura da câmera sai de cantos vistos inteiros num quadro SÓ (layoutPairHeight).
- Ainda falta:
  - Uma observação de CANTO multivista. A aresta vertical do canto j e os quatro vincos (2 de teto, 2 de piso) em
    cada quadro que o vê viram resíduos que tocam o mesmo vértice da planta.
  - O raio do giro (~0,3 m) e as estações de canto dão a paralaxe. Isso triangula o vértice e a altura juntos e
    tira o peso do vinco de piso isolado, escondido pelos móveis.

## 2026-09-27 - SALA EM L: TRÊS BUGS E O ESBOÇO DE PLANTA
- BUG 1 (app): o buffer de arestas dos pontos de fuga (PedgeRays/PedgeLabels -> Pedges) só era declarado, nunca
  alocado. vanishDetect rodava sem arestas, e TRoomLayout nunca recebia linhas. A planta AO VIVO nunca funcionou
  (rtLayout de 002325 e 003708: flags 0, câmera 1,45 padrão). Só o capInspect funcionava, porque ele aloca o buffer.
  Corrigido em OnCameraReady.
- BUG 2 (capLayout): o primeiro quadro fixava a âncora e o filtro de deriva (mediana dos últimos 5). 002325 teve o
  primeiro eixo em 68,8, contra ~63 real, e os 11 quadros seguintes do teto, coerentes entre si, foram rejeitados.
  Correção:
  - AddFrame guarda TODOS os quadros, com PframeAxis.
  - Solve rejeita pela mediana SIMÉTRICA dos vizinhos (±5 quadros, FrameKept).
  - AnchorDeg = mediana de todos os eixos; Solve compensa (âncora - mediana).
  - layoutMaxEdges 240000 -> 400000 (40 quadros x 8000 estouravam).
  Resultado: 002325 com 6 cantos (1 reflexo), 5,75 x 2,83; 003708 com 6 cantos, 3,65 x 3,40. As demais sessões
  seguem iguais.
- BUG 3 (capInspect): o contador de registros tinha 9 posições, e rtLayout = 9 estourava. A ferramenta agora
  imprime a planta do app ("app plan room").
- Usuário: "ao invés de pedir para ir ao canto #1, mostre a planta e indique onde estar e para onde apontar".
  Sem planta, o app agora desenha um ESBOÇO: quadrado nos eixos VP com 4 estações diagonais (PplanSketch), rótulo
  "esboço" em vez das medidas. Dica: "Fique no ponto amarelo, aponte para o anel e toque em Cheguei"; botão "Cheguei".
- AJUSTE DE FEIXE com cantos multivista + vinco do piso + elos do giroscópio (capMosaic): implementado, mas
  DESLIGADO por constantes (cBundleSigmaGyro 1000, cBundleFloorWeight 0, cBundleCornerWeight 0).
  - ACHADO: a atitude do giroscópio entre keyframes consecutivos discorda 1-4 graus das rotações dos pontos de fuga
    refinadas pela imagem. Suspeita: sincronia do carimbo de tempo do quadro com o do sensor (ou rolling shutter).
    Sem corrigir isso, o giroscópio não chega ao subpixel.
  - Com os elos em 0,002 rad, a sala inflou para 3,19 x 3,37. Sem eles o resultado oscila ±3% entre rodadas
    (202726: 2,96-3,13 x 3,06-3,26). O melhor anterior (2,995 x 3,186) não se reproduz após a mudança do eixo da
    planta para a mediana.
  - Cantos: só 2-6 observações por rodada. Piso: 7-37.
  - PRÓXIMO: sincronizar a atitude na exposição do quadro (app) e rever a estabilidade do feixe.

## 2026-09-27 - SALA EM L FECHADA (003708) - VINCOS DE QUADROS NIVELADOS
- As paredes distantes da sala em L só apareciam em quadros quase nivelados (pitch ~0). A regra antiga só aceitava
  vinco de quadros com pitch >= 5.
- Agora (layoutCreaseTop): a linha mais alta de um quadro do giro central vale como vinco se a sua elevação
  perpendicular for <= pitch + 25 graus. Assim há margem para uma linha mais alta ter sido vista. Quadros do teto
  (pitch >= 5) continuam aceitos em qualquer elevação; estações de canto nunca dão paredes.
- 003708: L completo com 6 cantos (1 reflexo), área total 5,94 x 5,85.
  - Braço principal: 3,65 x 5,85.
  - Ala: 2,29 x 2,70.
  - O vão é o ESCRITÓRIO (usuário: fica exatamente atrás): 2,29 x 3,15. Os 3,15 batem com a parede do escritório
    oposta à porta (3,00 por dentro + ~0,15 de parede; usuário: é a parede vista nos quadros 2 e 3).
  - 5 estações planejadas.
- 002325 (giro ruim, eixo inicial errado) cai para 4 cantos, 5,54 x 2,64. As demais sessões não mudaram.
- Escritório conhecido: 2,40 (oposto à janela) x 3,00 (oposto à porta), pé-direito 2,70. A sala usa 2,80 assumido.
  PENDENTE: com o pé-direito real, a escala muda (~-8% se for 2,70).

## 2026-09-27 - QUADRO "VAZADO" NA COBERTURA
- Usuário: um quadro fica vazado por mais que insista, mas o giro completa e avança. Isso confunde o operador.
- Os dados (002325, 003708) têm as 28 posições preenchidas.
- Causa provável: enquanto o worker processa a foto anterior (JPEG 12 MP + pontos de fuga, alguns segundos),
  Offer(allowKeep = false) não guarda nada. O quadro mirado fica vazado sem aviso. No ÚLTIMO quadro, a foto entra e
  stationDone troca de fase no mesmo instante, então o operador nunca o vê preenchido.
- Correção:
  - TSpinTracker::AimedEmpty(band, bin).
  - Mirado + vazio + ocupado: o quadro fica amarelo translúcido e a dica diz "Segure: processando a foto anterior".
- Também observado: as faixas do giro se dividem no horizonte (-42..0 e 0..+42). Uma foto a +1 grau já ocupa o
  quadro do teto, e erguer o celular depois não troca a foto. Candidato a rever: centrar a faixa do teto para
  enquadrar o vinco.
- Gravação de tela via adb (print a cada 2 s, 20 min) armada para confirmar na próxima captura.

## 2026-09-27 - CAPTURA 010618 (5 CANTOS)
- A planta AO VIVO funciona: o rtLayout do app tem flags 5, 6 cantos e 5 estações. A dedução do canto pela
  diagonal escolheu 5 estações distintas (registros seEnd: cantos 2, 3, 4, 5, 1).
  - O seBegin guarda só a sugestão do mapa.
  - O fim da última estação era gravado duas vezes; corrigido com Pphase = rpWalk antes de finishRoom.
  - A dica da fase de cantos saía cortada; encurtada para "Fique no ponto amarelo, mire o anel (k de n)".
  - O capInspect agora imprime begin/end das estações e conta os registros até rtLayout.
- Laranja: medidas ruins de verdade (referência ~41).
  - O piso diagonal (45) dá 86/77.
  - Poucas linhas de parede dão 33-35.
- Cinza: quadros sem linhas (teto ou parede lisa). O usuário notou os "blocos suspeitos" agora em cinza.
- PROBLEMA ABERTO (usuário: "a planta ainda não guarda a proporção convexa"): 010618 saiu 6,07 x 6,01 com o vão
  do L de só 0,93 x 0,35.
  - O caixa bate com 003708 (5,94 x 5,85). As duas paredes do vão (os lados do escritório) não geraram vinco
    daquele ponto do giro.
  - A parede w = 4,28 é vista até u = 1,85, além da parede u = 1,37. Isso prova que a u = 1,37 termina e existe
    um canto reflexo, mas o montador de polígono estendeu a u = 1,37.
  - PRÓXIMO:
    (a) o montador de polígono não deve estender uma parede além do alcance visto quando outra parede
        a contradiz;
    (b) usar as ESTAÇÕES DE CANTO para paredes: elas veem o vão de frente, mas a posição de cada estação precisa
        vir do feixe.
- Usuário: "ao me deslocar para um canto, a captura sugerida deveria ser a diagonal oposta". layoutStations agora
  exige o alvo a até 35 graus da bissetriz interna do canto (o mais distante entre esses). Só sem candidato cai no
  mais distante visível. Em 010618, a estação no canto 1 (nicho de 0,93 m criado pelo vão mal medido) continua no
  fallback: 36 graus. A causa ali é a planta, não a regra.

## 2026-09-27 - ESTAÇÕES DO L CONFORME O MAPA DO USUÁRIO
- Mapa do usuário, na numeração do app (horária, mantida a pedido): o canto que encara o reflexo mira as pontas
  dos DOIS braços, o reflexo também é estação e mira esse canto, e os demais miram na diagonal.
- layoutStations:
  - Todo canto, inclusive o reflexo (bissetriz virada para dentro), mira o mais distante dentro de ±35 graus
    da bissetriz interna.
  - Um canto convexo ganha uma 2a estação se outro canto do leque estiver a >= 25 graus da 1a mira e a >= 0,7
    da distância.
- 003708 (7 estações): 0->4, 1(reflexo)->4, 2->4, 3->5, 4->2, 4->0, 5->3. Bate com o mapa do usuário.
- 203334 (degrau espúrio do guarda-roupa) chega a 8 estações (o máximo). Retângulos seguem com 4.
- Usuário: "4 deveria capturar 0 e 2 (côncavos) mas também 1 (o convexo)". Regra nova: o canto para onde o reflexo
  mira ganha uma estação de volta para o reflexo (visto de frente). As estações são ordenadas por canto (estável).
  003708: 8 estações (0->4, 1->4, 2->4, 3->5, 4->2, 4->0, 4->1, 5->3). Limite layoutMaxStations = 8.
- Usuário: "os blocos em cinza deixam a interface confusa". Quadro sem linhas (capturado, não verificável pelos
  pontos de fuga) passa a ser AZUL-CLARO, e há uma legenda sob a vista em perspectiva: verde "ok", laranja "fora
  do eixo", azul "sem linhas".

## 2026-09-27 - REGRA DE OURO 15 (ordem do TODO)
- Usuário: item em operação no topo, pendentes no meio, resolvidos no final. Gravado em
  slices/regraOuro/15-ordem-do-todo.md e indexado. Lista refeita nessa ordem.

## 2026-09-27 - O "L" DE 012809, A VERTICAL POR QUADRO E A FUSÃO DE PAREDES (DESCARTADAS)
- Usuário: "o teto e seus pontos de fuga não devem estar conflitando".
  - Conta: d = (H - h)*cot(elevação do vinco). Para uma parede a 5 m (vinco ~14 graus), 1 grau de erro na
    vertical dá ~0,38 m.
  - A vertical do sensor difere da vertical medida pelas linhas em 0,1-5,3 graus POR QUADRO. O TTiltBias corrige
    só a média.
  - TESTE: vertical dos pontos de fuga por quadro (suporte vertical >= 300/800/1500) PIOROU as salas de medida
    conhecida (202726: 2,94 x 3,09 -> 2,78 x 2,92 com 6 cantos espúrios; 200900 oscila). DESCARTADO de novo;
    experimento removido.
- Usuário: "cantos 0-1 e 2-3 compartilham o mesmo ponto de fuga, e a distância 1-2 não deveria ser menor que 3 m".
  - 012809 dava um degrau de 0,22 m: parede das portas em u = 1,45 (quadros 8-10) e trecho rasante u = 1,66-1,86
    (quadros 13-14 da sessão, mesma parede vista quase de lado).
  - O lado u- veio de UM quadro só (5,23, pitch 4 graus) e o w+ de 4 quadros (4,6-5,0): as paredes distantes são
    as mais frágeis.
  - TESTE: fundir paredes paralelas do mesmo lado com degrau < max(0,4 m, 10%).
    - Por cadeia de vincos: desfez o L bom de 003708 (paredes intermediárias encadeiam).
    - Por pares lado a lado, ficando com a de fora: a 202726 caiu para 2,87-2,92 no lado de 3,20, porque duas
      frentes de armário (1,3 e 1,59) se fundem e somam peso sobre a parede real (1,73).
    DESCARTADO; o agrupamento original (0,08 m / 4%) foi restaurado e as linhas de base conferidas.
- O vão do escritório em 012809 NÃO aparece no giro central. PRÓXIMO: paredes pelas estações de canto
  (resseção pelas paredes conhecidas, depois paredes novas).
- Regra 15 aplicada à lista de tarefas: em operação no topo, pendentes no meio, resolvidos no final.

## 2026-09-27 - PAREDES PELAS ESTAÇÕES DE CANTO (1a versão)
- TRoomLayout::AddFrame recebe o índice da estação (0 = giro central; PframeStation).
- Solve:
  - Depois das paredes do giro central, cada estação é localizada pelas paredes conhecidas (layoutResect, por eixo).
    Cada vinco propõe W - s*d; vence a proposta com mais vincos casando uma parede conhecida (tolerância
    max(0,15 m, 6%)), com pelo menos 2 casamentos por eixo.
  - Estação fora das paredes conhecidas é rejeitada.
  - Os vincos da estação entram como se vistos do ponto do giro (offset su + s*d, faixa deslocada), e as paredes
    são refeitas. plan.stationLines conta esses vincos.
- RESULTADO nas sessões atuais: quase nada entra (0 vincos; 012809 tinha 1 estação localizada FORA da sala).
  - Os leques de canto eram nivelados (pitch -2 a -10): o teto aparece pouco e só as paredes a 6 m (0-5 vincos
    por estação, quase sempre de um eixo só).
  - Correção na captura: o leque do canto passa a mirar +12 graus (cCornerPitchDeg, faixa aceita -5..+29) e a dica
    diz "Incline para cima (linha do teto)". O desenho do leque (banda única) segue cobrindo a parede inteira.
- Linhas de base inalteradas (202726 2,94 x 3,09; 003708 5,94 x 5,85 com 6 cantos).
- capTest: o L sintético agora espera 8 estações (mapa do usuário). 0 falhas.
- Achado de ferramenta: o capTest usa o cache do abtestcache (build/Release/.abtestcache); para ver o detalhe de
  uma falha, apague o cache e rode de novo.
- APK instalado. PRÓXIMO: recapturar a sala em L com os cantos inclinados e medir quantas estações se localizam.

## 2026-09-27 - CORES DA COBERTURA SIMPLIFICADAS
- Usuário: "três cores no preenchimento é bem confuso; qual a razão do cinza?". O cinza era o quadro VAZIO: branco
  quase transparente sobre o fundo escurecido.
- Agora:
  - Vazio.
  - VERDE = capturado, inclusive sem linhas.
  - LARANJA só para quadros a mais de 8 graus do eixo (cOffAxisShowDeg: piso diagonal, linha espúria).
  - Amarelo = processando.
  - A legenda saiu. O veredito fino (±2 graus, sem linhas) continua nos registros.
- Na interface, isso substitui o "verde só depois da checagem de coerência": o operador não pode refazer um
  quadro, então só o que falta fotografar importa para ele.
- Usuário: "existe o quadro vazado e existe também o preenchido com cinza".
  - VAZADO: com 2 faixas, o desenho pintava do teto até yCeil e de yFloor até o piso; a faixa do horizonte nunca
    tinha cor. Agora as duas faixas se encontram no horizonte (inner = cy).
  - CINZA: era o quadro ainda não fotografado (branco translúcido sobre o fundo escuro). Agora aparece só com
    CONTORNO.
  - Na tela: contorno = falta, verde = capturado, laranja raro, amarelo = segure.
- Usuário: "as fotos em laranja podem ser recapturadas, e se o novo snap der verde ele substitui o anterior".
  - TSpinTracker::Reopen(band, bin): o bin continua contando como preenchido, mas a próxima vista firme dele é
    guardada (svKeep).
  - O app reabre um bin quando o veredito sai a mais de 8 graus do eixo (binOffAxis), até appMaxRetakes = 2 vezes
    por bin, para um piso diagonal não gerar fotos sem fim.
  - A cor e o eixo do bin seguem sempre a foto mais recente.
  - A foto antiga continua no arquivo da sessão (append-only), fora da planta (FrameKept) e das paredes (> 12 graus).
- Usuário (versão adaptativa): "o verde é definitivo; o laranja é sempre passível de atualização, mas permite ao
  operador encerrar a fase se desistir".
  - PbinOrange guarda o veredito NA CHEGADA: verde nunca muda depois, mesmo que o eixo do cômodo se mova.
  - Laranja reabre o bin SEM limite (appMaxRetakes removido).
  - checkComplete: a estação só fecha sozinha com todos os bins capturados, todos os vereditos chegados e nenhum
    laranja. Chamado no OnFrame e no worker, depois de gravar a imagem.
  - Com laranja sobrando, a dica diz "N em laranja: aponte para refazer ou toque em <botão>", e o operador aceita
    como está pelo botão.

## 2026-09-27 - O "DENTE" DE 070542, O MAPA COM OLHO E ALVO
- 070542 (L com cantos inclinados): a planta do app tinha 8 cantos. O usuário: "um dente que não existe no canto
  oposto ao convexo".
  - Diagnóstico com `capInspect --creases` (novo: TLayoutPlan.creases lista cada vinco com quadro, estação,
    lado, distância, faixa e peso). O dente era u = 2,25, vindo de UM quadro (quadro 1, peso 75).
  - "Dois quadros por parede" DESCARTADO: apagou paredes distantes reais, vistas por um quadro só (003708 perdeu
    o L; 070542 perdeu o fundo do braço).
  - Regra final: uma parede que não é a mais forte do seu lado precisa de >= 2 vistas OU de
    peso x distância >= 250 (cLoneWallWeightM). Dente: 169; paredes reais sozinhas: >= 367.
  - 070542 agora: L limpo com 6 cantos (completo), 6,78 x 7,19, vão 3,23 x 3,72. As medidas estão infladas em
    relação a 003708 (5,94 x 5,85); a escala e a vertical seguem pendentes. Demais sessões inalteradas.
- Estações de canto em 070542: 0 vincos aproveitados (a resseção ainda não localiza as estações).
- Mapa dos cantos (usuário, com o olho.png): OLHO LARANJA onde ficar, desenhado em vetor (drawEye) e girado para
  o alvo, com tracejado laranja fino; CÍRCULO AMARELO grande e vazado no canto a fotografar.
  - O ponto do giro virou uma cruz branca (o amarelo é do alvo).
  - Estações feitas em verde, pendentes em branco; a atual é desenhada por cima.
  - Dicas: "Vá ao olho laranja e mire o círculo amarelo (k de n)" / "Mire o círculo amarelo do mapa e pare".

## 2026-09-27 - DEGRAU SÓ CONFIRMADO; CANTOS NA HORIZONTAL
- Origem do dente de 070542: quadro 2 da sessão (planta 1). A moldura da parede u+ foi vista de viés perto do
  canto, e só uma linha fraca (peso 75) foi detectada, com elevação 30 graus. Se a parede está a 1,51 m, como os
  quadros 6-9 mostram de frente, deveria ser ~41 graus. Essa leitura deu 2,25 m.
  O teto desmente: o vinco forte w+ termina em u = 1,73, junto da parede verdadeira.
- Princípio do usuário: "um dente é confirmado em múltiplas capturas do mesmo acidente; todo outro caso é
  descartado". Regra final para a parede SECUNDÁRIA de um lado (a mais forte sempre fica); ela fica se:
  - foi vista por >= 2 keyframes (layoutMinWallViews); OU
  - um único keyframe viu >= 1,5 m dela (cLoneWallLenM: paredes do fundo, vistas de longe uma vez); OU
  - um vinco perpendicular termina nela, a menos de 0,35 m (cCornerMeetM: os vincos do teto se encontram nos cantos).
- Tentativas descartadas no caminho:
  - só 2 vistas: apagou as paredes do fundo;
  - só o encontro dos vincos: apagou o vão de 070542 (0,43 m de folga, contra 0,52 m do dente);
  - peso x distância: heurística sem física.
- Resultados:
  - 070542: L limpo com 6 cantos, completo, 6,78 x 7,19.
  - 003708: L intacto, 5,94 x 5,85.
  - 010618: perdeu o falso vão pequeno.
  - 012809: ainda com um degrau de 0,22 m (parede vista em trecho longo). Pendente.
  - 202726 e 200900 inalterados.
- Diagnóstico: capInspect --creases lista os vincos. As paredes mostram "views" (TLayoutPlan.wallViews, creases[]).
- Cantos de volta à HORIZONTAL (usuário: "pose natural, visada frontal do canto, sem teto nem piso"):
  - cCornerPitchDeg = 0.
  - Ao inclinar, a grade da vista fica VERMELHA e a dica diz "Deixe a câmera na horizontal, de frente para o canto".
  - A grade volta ao normal ao nivelar.
- Usuário: as visadas horizontais dos cantos dão perspectiva de pé-direito e verga (topo de porta, 2,10 m) para um
  refinamento milimétrico. É o caminho de escala até o LiDAR do iOS. PRÓXIMO candidato: medir pé-direito e
  vergas pelas estações de canto.

## 2026-09-27 - PÉ-DIREITO PELO CRUZAMENTO DE HIPÓTESES (verga 2,10 x típicos 2,70 / 2,80)
- Usuário: "um pé-direito típico tem 2,80 (no meu apartamento 2,70) e uma verga tem 2,10; essas hipóteses podem ser
  cruzadas".
- TRoomLayout::Solve = solveAt na suposição, depois o cruzamento. Se as vergas medem um pé-direito implícito
  (impliedCeilingM = suposição x doorScale), o candidato mais próximo entre {suposição, 2,70, 2,80}, dentro de 6%
  (cCeilingSnapFrac), vence.
  - Se vencer um típico diferente da suposição, a planta é ESCALADA (f = novo/suposto: vértices, paredes, vincos,
    altura da câmera, área x f^2). Nunca é refeita: refazer em 191624 mudou a altura da câmera e a forma.
  - Verga longe de todos (200900: 3,07) não muda nada.
- Resultados:
  - 202726 (real 3,00 x 3,20): 2,98 x 3,14 (-0,7% / -1,9%), o melhor até agora.
  - 203334 e 191624 foram para 2,70.
  - 175043 fica em 2,80 (verga 2,79).
  - As capturas da sala em L não acharam verga.
  - capTest: sala de 2,60 com suposição 2,60 fica; suposição 2,80 é escalada para 2,70. 0 falhas.
- PASSO FUTURO (usuário): mesas criam uma falsa linha de piso, paralela à verdadeira mas defasada na altura
  (estantes criam falso teto, e esse caso já é resolvido pela linha mais alta). Confirmar o vinco do piso pela
  consistência do PADRÃO DE AZULEJOS do piso (juntas, módulo) para desempatar.

## 2026-09-27 - AVISO DE PORTA ABERTA (recomendação)
- Usuário: "uma porta aberta cria ruído; fechada, a verga fica mais realista. Se detectar uma porta aberta, oriente
  a fechá-la. Não é erro, é recomendação".
- layoutOpenDoors (Solve): cada aresta acima do horizonte é posta na altura da verga (a medida, ou 2,10). Vota se:
  - cai numa faixa de 0,3-1,0 m saindo de uma parede de orientação perpendicular;
  - não está sobre outra parede (0,15 m);
  - fica dentro da extensão da parede.
  O pico precisa de >= 40 votos e de 5x o fundo de arestas soltas da parede (sem o contraste, todas as sessões
  davam 4 portas). plan.openDoors / openDoorWall / openDoorAt.
- Resultado: 2 alarmes em 10 sessões.
  - 002325: parece real (quadro 6, passagem com a porta aberta).
  - 012809: parede da TV, provável barra da persiana (falso).
- App: faixa laranja "Porta aberta? Feche-a para uma captura melhor" nas fases de caminhar e cantos.
- PRÓXIMOS critérios (usuário), para filtrar a folha:
  - BATENTE: a folha aberta nasce num batente (vertical do vão).
  - MAÇANETA: fechada, fica dentro do retângulo da verga, sem ortogonal; aberta, fica perpendicular ao batente,
    oposta à dobradiça.
  O layout hoje só guarda arestas horizontais; os dois critérios pedem as verticais (batentes) por quadro.

## 2026-09-27 - ATITUDE POR QUADRO, RESOLUÇÃO MÁXIMA, LIMPEZA
- Usuário: "cada frame deve registrar o vetor perpendicular da tela que fez a captura e a rotação em torno dele,
  com a gravidade como zero".
  - TFrameMeta (LIDARCAP no JPEG) cresceu de 300 para 320 bytes: forward[3] (mundo, y para cima) + rollDeg.
  - geomRollDeg: ângulo de "cima do mundo" (projetado no plano da imagem) até o +y da câmera; horário visto pela
    câmera; NaN a menos de ~8 graus do zênite ou do nadir.
  - Decode aceita os blocos v1 de 300 bytes e deriva os dois valores da pose.
  - capInspect: colunas fwdX, fwdY, fwdZ, rollDeg no frames.csv. 070542: roll -82..-94 (celular em pé; o sensor é
    deitado), fwdY = sen(pitch) confere.
- Usuário: "a maior resolução possível da câmera".
  - Moto g(9) play: sensor principal 4016 x 3016; a maior saída YUV (e RAW) é 4000 x 3000, que o app já usava.
  - appMaxPixels 12,6 MP -> 50 MP para outros aparelhos pegarem o maior 4:3.
- Usuário pediu para esvaziar as capturas antigas. As 14 sessões do celular (1,0 GB, 26/09 17:32 a 27/09 07:05)
  foram APAGADAS com `rm -rf .../files/imovel_*`. O usuário esvazia as do PC.

## 2026-09-27 - MÉTRICA PRINCIPAL: ÁREA DE VASSOURA
- Usuário: a distância até o piso (LiDAR) desempata falsos positivos como a mesa da sala; a área de vassoura é a
  informação mais desejada do imóvel, e quanto mais precisa, mais robusta no mercado. Registrado em
  slices/projeto/missao.md como critério de decisão.

## 2026-09-27 - PASSO FUTURO: GRADE DE AZULEJOS (FFT) E REMOÇÃO DE MÓVEIS
- Ideia do usuário: os azulejos formam uma grade identificável por FFT; com o padrão, fazer um "flood fill"
  estendendo o piso e as paredes para remover os móveis do ambiente.
- Plano:
  1. Ortofoto do PISO (vista de cima), pela mesma retificação das paredes: pontos de fuga + altura da câmera.
  2. FFT 2D: os picos dão o módulo (45/60/80 cm) e a orientação da grade. O piso diagonal a 45 graus, que hoje
     engana os pontos de fuga, vira informação.
  3. Ganhos:
     - a linha do piso é confirmada pelas juntas que terminam na parede (desempate da mesa);
     - o módulo padrão do azulejo é uma régua a mais para a escala e a área de vassoura;
     - onde a grade quebra, há um móvel ou tapete: mapa de oclusão.
  4. Remoção dos móveis:
     - piso: síntese periódica a partir de um trecho limpo, preenchendo as oclusões;
     - paredes: tinta uniforme já equalizada;
     - as vistas dos cantos cobrem a paralaxe.

## 2026-09-27 - VISTA DO PISO EM CADA CANTO, CENTROS ADAPTATIVOS
- Usuário: em cada canto, depois das três poses do canto oposto (horizontais), capturar também o centro do cômodo
  NITIDAMENTE PARA O PISO: a malha de ladrilhos vista de cada canto (base para remover os móveis) e um fator a
  mais de conferência.
- Implementação:
  - TSpinConfig::ForFloorView: 1 bin, faixa de pitch -35 ±17, largura de meio quadro. TSpinTracker::AimFan
    prefixa o centro do leque.
  - App: checkComplete, depois do leque do canto, chama beginFloorView: mesmo canto, rastreador novo mirando o
    alvo; os quadros levam spinBand = appFloorBand (1).
  - Os vereditos do leque e da vista do piso não se misturam (floorFrame == PfloorView). A vista do piso nunca
    fica laranja (um piso diagonal a encheria).
  - Dicas: "Agora mire o centro do cômodo, olhando para o piso" / "Incline mais para o piso". A grade vermelha
    só vale para o leque nivelado.
  - O mapa põe o círculo amarelo no centro-alvo.
- Usuário: "o centro é adaptativo: um retângulo tem um centro, o L três ou mais (um por perna e o cruzamento)".
  - layoutCenters: as coordenadas das paredes cortam a planta em células; cada célula dentro do cômodo tem um
    centro. plan.centers; o esboço tem o centro (0,0).
  - Cada canto mira o centro MAIS PRÓXIMO (usuário: não todos; um fica atrás da parede saliente).
  - capTest: retângulo 1 centro, L 3. 0 falhas.
  - OPÇÃO FUTURA (usuário): vista extra para o CRUZAMENTO, visível de quase todos os cantos.

## 2026-09-27 - DORMITÓRIO 081054, MAPA DE NAVEGADOR, ALERTAS NA GRADE
- 081054 (dormitório, porta aberta de propósito):
  - planta 2,98 x 3,21 m com 4 cantos, 4 estações (altura da câmera 1,52 solved);
  - a verga implica 2,52: longe de 2,70 e 2,80, nada muda;
  - piso em DIAGONAL (45 graus).
- Achado 1: o piso diagonal dava laranja na faixa de baixo. Com a retomada automática, o mesmo bin foi
  fotografado ~9 vezes (quadros 15-26). binOffAxis agora trata um eixo a 45 ±6 graus (cDiagonalTolDeg) como padrão
  diagonal do piso: fica verde.
- Achado 2: o detector de porta aberta NÃO serve sem as verticais. No 081054, todo pico fica só ~1,8x acima do
  fundo (700-1400 votos por posição); a porta real não se destaca. O aviso foi RETIRADO da tela. layoutOpenDoors
  segue como diagnóstico no capInspect até os critérios de batente e maçaneta (tarefa pendente) usarem arestas
  verticais.
- Mapa: o olho fica fixo embaixo (85% da altura), no meio; a planta gira em volta dele (heading-up) e escala pela
  maior distância do olho até um canto. As linhas são recortadas na caixa (clipToBox, Cohen-Sutherland). O olho
  mantém sempre a mesma cor e posição (usuário).
- Alertas de pose SÓ na grade de captura (usuário: "a grade é bem ruidosa e serve para isso"):
  - o leque aceita ±10 graus do nível (cCornerLevelHalfDeg);
  - fora disso, fundo e painéis ficam VERMELHOS;
  - na vista do piso a regra se inverte: vermelho quando o celular NÃO está inclinado para o piso.
  - A borda vermelha do mapa foi retirada.

## 2026-09-27 - ALERTA DE POSE TAMBÉM NO GIRO CENTRAL
- Queda de energia às ~08:25: nada perdido (APK de 08:23 já instalado; 13 arquivos sem commit intactos).
- Usuário: em vermelho, NENHUMA captura é registrada; o giro central segue a mesma regra dos cantos.
  - TSpinTracker::GuidedBand = a faixa mais alta com bin vazio (teto primeiro); Offer devolve svOffBand fora
    dela (antes da checagem de velocidade). No Moto as faixas se encontram no horizonte: no giro do teto o
    celular vai no máximo à horizontal, sem apontar para o chão; no giro do piso, o oposto.
  - Com todos os bins preenchidos, qualquer faixa vale (o laranja pode ser refeito onde estiver).
  - PoseAllowed() (última inclinação dentro da faixa guiada) pinta a grade de vermelho no centro e nos cantos.
  - Nos cantos, o MAPA também fica vermelho (fundo e paredes); o OLHO continua laranja e fixo embaixo.
  - Dica no centro: "Incline para cima (linha do teto)" / "Incline para baixo (linha do piso)".
- capTest: novo caso da ordem das faixas (headPitchPose subiu para o topo do arquivo). 0 falhas. APK instalado.
- Porta aberta: o falso negativo do 081054 SEGUE PENDENTE (critérios de batente e maçaneta pedem as arestas verticais).
- CAPTURA 083920 (usuário: "os alertas em vermelho foram incisivamente PERFEITOS"): 45 imagens em 230 s. Giro do
  teto: 14 quadros, pitch +11,7 a +26,9. Giro do piso: 15 quadros, -10,3 a -35,8. Nenhum quadro na faixa errada.
  4 cantos com leque nivelado (-3,7 a -9,1) + vista do piso (-31,9 a -43,9).
  - Planta ao vivo: flags 7, 4 cantos e 4 estações. capInspect: 3,74 x 3,46 m, 12,91 m2, câmera a 1,52 m.
  - Verga implica 2,51 m: nada muda.
  - Os 13 laranjas são o piso diagonal: o eixo cai em ~10 graus, 55 - 45, e a tela mostra verde.
  - Estações de canto: 0 vincos. Os leques são nivelados, então isso é esperado.
  - O bin 7 do piso foi guardado 2 vezes: quadros 14 (-10,3) e 15 (-24,8), ambos alinhados. A causa não foi
    investigada.

## 2026-09-27 - BORRÃO PELA FFT (capBlur)
- Usuário: "FFT na imagem evidencia o grau de borrão... registrado no metadado; nova captura (mesmo verde) com
  borrão menor substitui a anterior; um piso de borrão deixa a captura em laranja". O autofoco leva um tempo
  para convergir.
- core/capBlur:
  - A luma é reduzida 4x por média de blocos. Em resolução plena, o grão e o sharpening do ISP (1 a 2 px)
    enchiam a banda alta, e o frame 44, fora de foco, lia 0.
  - Grade de 3x3 blocos de 256 px, cada um com janela de Hann e FFT 2D radix-2.
  - Potência média na banda baixa (0,03-0,06 c/px) e na alta (0,10-0,20), descontado o piso de grão
    (0,40-0,50).
  - A razão entre as bandas, comparada à de uma cena 1/f², dá o sigma gaussiano equivalente.
  - Blocos lisos (desvio < 6) ficam fora. A medida do quadro é a MEDIANA dos blocos: com o mínimo, uma única
    aresta forte dava 0.
- Calibração em 083920, pelo julgamento do usuário:
  - 42 excelente: 0,0.
  - 32 quase bom: 2,6.
  - 44 e 15 fora de foco: 5,7 e 6,3.
  - Sintético: 3 caixas de 13 px (sigma 6,5) medem 6,44.
- LIDARCAP: blurPx (mediana) e blurMinPx em 2 WORDs (centipixels + 1; 0 = não medido) nos 4 últimos bytes do
  bloco de 320.
- App (worker, na luma crua antes do JPEG):
  - Acima de cBlurMaxPx 4,0: LARANJA, reabre sem limite. Vale também na vista do piso.
  - Verde acima de cBlurGoodPx 2,0: reabre até appBlurTries = 2 vezes. A nova foto só substitui (eixo,
    borrão, cor) se for mais nítida.
- capInspect: `--blur` mede no JPEG e imprime por bloco. frames.csv ganhou blurPx, blurMinPx e blurTextured.
- PENDENTE: nas retomadas, o arquivo guarda todas as fotos (append-only). A etapa offline deve escolher a mais
  nítida por bin.
- CMake/VS: o `build/norm.sh` regravava o CMakeLists a cada checagem, e o Visual Studio (Abrir Pasta) reconfigurava
  e abria a janela de saída. Agora o script pula o CMakeLists e só regrava arquivo que precisa de CRLF.
- capTest 0 falhas; APK instalado.

## 2026-09-27 - PORTAS COMO RÉGUA (capDoor), BOTÕES PROTEGIDOS
- Usuário: pé-direito típico 2,80 (no apartamento dele 2,70), verga 2,10; as medidas saem um pouco acima do real.
  Diagnóstico:
  - o cruzamento de hipóteses (Solve) existia, mas a "verga" do histograma dava pé-direito implícito de 2,51-2,52
    (081054, 083920), fora da janela de 6%, então ficava 2,80;
  - em 083920 a linha vencedora era a divisória do armário embutido, não a porta.
- Critérios do usuário para a porta:
  - acima da verga, a cor é a da parede (não a do armário);
  - maçaneta e batente desempatam;
  - a porta quase sempre fica perto de um canto;
  - batente e verga medem na mesma vertical, sem escala.
- core/capDoor:
  - doorFrontal: rotação pura para a vista frontal nivelada da parede, a partir do YUV do quadro. Numa vertical da
    parede, as alturas ficam proporcionais às linhas da imagem, se a vista estiver nivelada.
  - doorDetect: batentes = corridas verticais longas.
  - Par de batentes: verga = o topo mais baixo, piso = o pé mais alto. A folha aberta, mais perto, sobe acima da
    verga e desce abaixo do pé (frame 30).
  - A verga precisa atravessar o vão. Rejeita se:
    - a linha continua ao lado do alizar (divisória);
    - os dois batentes seguem acima dela (frestas do armário);
    - há arestas na parede acima;
    - a cor difere da parede ao lado.
  - Maçaneta e canto próximo entram como bônus.
  - Saída: vinco/verga e câmera/verga. TDoorStats conta as rejeições e lista os batentes.
- capInspect `--doors`: door_NNN_paredeK.bmp com as marcas; imprime candidatos e batentes.
- 083920:
  - a porta foi achada nos frames 30 e 31 (mesma porta, estação 1): vinco/verga 1,409 e 1,404; câmera/verga 0,645 e
    0,668;
  - com a verga em 2,10: linha mais alta da moldura ~2,96 m, borda de baixo ~2,89, câmera 1,35-1,40;
  - isso NÃO bate com o pé-direito de 2,70 (a medida à mão só com o pitch dava 1,307 para a borda de baixo): falta a
    verdade de trena deste cômodo;
  - o armário (frame 41) ainda passa como candidato; sem vinco visível, não dá razão.
- Fluxo decidido pelo usuário:
  - a pré-análise gera candidatas;
  - depois dos cantos, uma estação de PORTAS: o operador fotografa cada candidata de frente e confirma ou descarta;
  - vale a porta no CENTRO da imagem (as vizinhas ficam de fora);
  - o quarto tem 1 porta, a sala 3.
  - Implementação: PENDENTE.
- App:
  - "Concluir cômodo" (logo acima de "Cheguei") desabilitado até todas as estações entrarem;
  - voltar do sistema exige 2 toques em 3 s, com faixa vermelha "Voltar de novo encerra o cômodo/imóvel". A captura
    não é retomável.
- Maçaneta (usuário: "estando aberta, a maçaneta ficou na folha perpendicular"): cada batente é procurado dos dois
  lados, na faixa de altura da própria corrida (a folha aberta, mais perto, aparece mais alta e mais baixa que o vão).
  Para fora do vão, a busca vai até 1 largura (a maçaneta fica na borda livre). doorKnob agora recorta os limites.
  083920: frames 30 e 31 com knob 1; o frame 31 também com canto.
- APK instalado com as travas de botão e do voltar.
- Trena (usuário): folha da porta com 2,10; o batente sobe mais ~5 cm (topo do batente ~2,15). O detector mede o
  topo do VÃO (a borda de baixo do batente), então o 2,10 da folha é a referência certa.
- Sensibilidade à inclinação (capInspect --door-tilt): o eixo x da câmera no retrato é quase vertical, então girar
  nele mexe pouco. Pela conta, 3 graus de pitch mudam a razão vinco/verga em ~1,4%. A inclinação residual NÃO explica
  1,409 contra ~1,31 (2,70 + moldura). Sobram:
  (a) o pé-direito deste cômodo é maior que 2,70 (a moldura estaria a ~2,9);
  (b) o piso achado no pé do batente (linha 858) está alto;
  (c) a planta está errada na altura da câmera: a razão câmera/linha do teto dá 0,456 pela porta e 0,543 pela planta
      (1,52/2,80).
  Com (a), as medidas da planta cresceriam, e o usuário diz que já estão acima do real. Falta o pé-direito de trena.
- Trena: pé-direito do apartamento 2,70; folha 2,10 (batente +5 cm). ERRO ACHADO: o piso vinha do pé mais alto dos
  dois batentes. No frame 30, a borda do alizar perde contraste em 858 (fundo escuro do corredor, depois o piso
  claro dele); a borda da dobradiça da folha aberta está no plano da parede e vai até o piso verdadeiro (1002).
- Regra nova:
  - verga = o topo mais baixo do par;
  - piso = o pé mais baixo de um batente que nasce na verga;
  - daí em diante as proporções usam o VÃO (piso - verga);
  - a caixa "acima da verga" fica abaixo da moldura;
  - doorRunGap passa de 8 para 24;
  - plausibilidade: câmera entre 0,45 e 0,95 vão, linha do teto entre 1,1 e 1,8 vão.
- 083920:
  - frame 30: linha do teto 2,78, câmera 1,51;
  - frame 31: 2,77 e 1,55 (esperado ~2,75 = 2,70 + projeção da moldura; câmera da planta 1,52);
  - frame 29: 3,03 (bordas curtas nos dois batentes, piso alto);
  - REGRA DE TRÊS (usuário): a mesma porta em vários quadros; a mediana descarta o ruim (1,322 -> 2,78 m).
- DIRETIVA (usuário): pé-direito único no imóvel (exceção: banheiro rebaixado, sala com mezanino), registrado em
  slices/projeto/missao.md. Mezanino adiado.
- PRÓXIMO:
  (1) Solve: com portas, linha do teto = 2,10 x mediana(vinco/verga); valor do IMÓVEL para cômodos sem porta;
  (2) portas no worker do app (quadros de canto nivelados) + estação de portas com confirmação pelo centro da imagem.
- Celular esvaziado a pedido do usuário (nova sessão): 081054 e 083920 foram antes trazidas para
  build/sessions/pull/ (a pasta do PC estava vazia; tamanhos conferidos byte a byte) e então apagadas no aparelho.

## 2026-09-27 - SESSÃO 093519 (ESCRITÓRIO + DORMITÓRIO): FOCO, EXCEÇÃO DO LARANJA, PISO ANTECIPADO
- Escritório: 305 fotos em 1 cômodo (antes 45); 279 com borrão > 4 px. Isso NÃO era erro da métrica: comparei, por
  engano, a mediana do app com o MÍNIMO do JPEG; mínimo contra mínimo, app e JPEG batem (4,30/4,26, 5,23/5,23).
  Somar grão (σ 2-4) ao JPEG não muda nada.
- CAUSA: o sensor principal do Moto tem MANUAL_SENSOR, então o app desligava o AF e fixava 0,5 D. A calibração de
  foco é APPROXIMATE (hiperfocal 0,245 D): o foco fixo errava e borrava tudo por igual.
- CORREÇÃO:
  - Porta ganhou Autofocus() (bool: false sem lente manual) e o evento OnFocus(travado, dioptrias).
  - TAndroid: AF_MODE_AUTO + TRIGGER_START; resultados observados até FOCUSED_LOCKED/NOT_FOCUSED_LOCKED ou 2,5 s;
    depois AF_MODE_OFF com LENS_FOCUS_DISTANCE onde parou.
  - OnFocus sai FORA do lock do adaptador (o app chama Autofocus segurando o dele: evita ABBA).
  - App: requestFocus no início de cada estação e da vista do piso; sem keyframe enquanto foca (limite 4 s); dica
    "Focando: segure firme".
- Usuário: no giro do piso, olhar para cima fica vermelho, EXCETO sobre um quadro laranja da faixa de cima (refoto
  legítima); verde segue vermelho. TSpinTracker::Reopen(band, bin, wrong): wrong = laranja, pode ser refeito fora da
  faixa guiada; a reabertura por nitidez não abre essa exceção. capTest cobre os dois casos.
- Usuário: nos cantos, apontar para baixo não fica vermelho se mirar o centro esperado do cômodo. A vista do piso é
  capturada ali (Pahead, rastreador ForFloorView mirando floorViewHeadingDeg) e o passo seguinte é pulado
  (PaheadDone). Mirando fora do centro: vermelho e nada é guardado. Dicas: "Segure: foto do piso adiantada" /
  "Para o piso, mire o centro do cômodo". PposeOk decide o vermelho da grade e do mapa.
- Sessão 093519 APAGADA do celular a pedido do usuário (a pasta do PC também foi zerada por ele). APK instalado
  09:59:52.
- PENDENTE: gravar a distância de foco por quadro (intrínseca); a vista do piso antecipada usa o foco do leque.
- Recaptura do escritório (100303, 169 fotos): o AF travou em 0,05 / 2,18 / 0,26 / 0,26 D (log "focus locked"). O
  borrão ficou ~5 px mesmo nas estações de 0,26 D (plausível): no giro central, 11 de 131 fotos com <= 2 px. Planta
  1,60 x 2,11 (real 2,40 x 3,00), câmera 1,81.
  - Frame 157: véu no quadro inteiro + raio de luz diagonal saindo da janela. Suspeita de LENTE SUJA (digital); o
    usuário foi orientado a limpar.
- Usuário: AF pelo centro, longe das bordas (móveis, monitores); depois "o triângulo que toca os dois cantos de cima
  e o centro", porque o teto é limpo e o piso atrapalha.
  - Porta: Autofocus(rects, count), retângulos normalizados na imagem NATIVA, o mais importante primeiro.
  - TAndroid: ACAMERA_CONTROL_AF_REGIONS (matriz ativa, peso 1000), tantos quanto CONTROL_MAX_REGIONS[AF] permite.
  - App: 3 faixas do triângulo (0,17-0,32 primeiro; 0,02-0,17; 0,32-0,47), meia-largura pela geometria do
    triângulo, convertidas pela rotação do sensor.
  - Resultado fora de 0,1..1,6 D (10 m..0,6 m): uma nova tentativa (appFocusTries 2, zerado por estação).
- APK instalado.
- Usuário: com o triângulo do teto, "a parte do teto ficou perfeita e esculhambou o piso". Pediu autofoco
  ADAPTATIVO: teto = triângulo ancorado em cima; piso = triângulo invertido; horizontal = só o centro.
  - TFocusAim (faCeiling / faLevel / faFloor); requestFocus(aim).
  - Giro central: começa em faCeiling e refoca quando a faixa guiada muda (PfocusBand): faFloor no giro do piso.
  - Leque do canto: faLevel (30% central). Vista do piso: faFloor.
  - A nova tentativa (resultado fora de 0,1..1,6 D) repete a mesma mira.
  - Moto: CONTROL_MAX_REGIONS AF = 1, só a faixa principal de cada triângulo é usada.
- APK instalado.
- 102600 (escritório com AF adaptativo): o usuário achou "bem melhor". Planta 3,13 x 2,34 com 2,80 suposto (real
  3,00 x 2,40; com 2,70 daria ~3,02 x 2,26). O borrão segue ~5 px em todas as faixas: em 100% as bordas espalham
  2-3 px. É o limite da ótica do Moto (48 MP binado em 12). Frame 068 ainda com raio de luz a partir da janela
  (sol direto na ótica).
- LOG DO AF suspeito: a 2a tentativa trava 57 ms após a 1a, e vários travamentos saem em 0,00-0,02 D. Rever se o
  estado do AF lido é do pedido anterior (falta checar se o resultado pertence ao disparo).
- Borrão RELATIVO à câmera:
  - laranja acima de max(4 px, 1,3 x mediana das últimas 64 fotos da captura);
  - troca por mais nítida acima de max(2 px, 1,1 x mediana);
  - sem veredito antes de 5 amostras.
  - addRoomBlur / blurFloorPx; o histórico NÃO zera por cômodo (a maciez é da câmera).
- APK compilado, NÃO instalado: o adb por Wi-Fi recusou a conexão (10061). O modo tcpip se perdeu; é preciso USB
  para `adb tcpip 5555` de novo.
- USB religado: adb tcpip 5555 + connect 192.168.15.22:5555 OK; APK do borrão relativo instalado.
- Dormitório capturado com borrão relativo e AF adaptativo: "foco bem coerente, nenhum bloco laranja" (usuário).
- LINHAS DE APOIO DA GRADE (usuário: "devem encaixar nos cantos do cômodo... se o sprite não acompanhar os cantos,
  se torna um ruído"):
  - a posição teórica (eixo + 45 + 90k) SAIU;
  - giro central: updateGuides após cada quadro do centro. Playout.Solve com o que já entrou (as paredes vêm mesmo
    com falha pfFewWalls); uma parede u e uma w cujas extensões se alcançam (0,35 m) formam uma quina, e a linha vai
    no rumo dela a partir do ponto do giro;
  - côncava (as paredes voltam para o ponto do giro): vincos "\ /" para o teto e "/ \" para o piso;
  - convexa (saliente): a forma invertida;
  - estação de canto: uma linha só, no canto-alvo da planta (convexa se o vértice-alvo é reflexo);
  - sem quina conhecida, nada é desenhado.
- REGRA DE OURO 16 (usuário): baixou e conferiu, apaga do celular (a capacidade dele é limitada).
- APK das linhas de apoio compilado, NÃO instalado: o adb por Wi-Fi caiu de novo (10061). Pede USB e `adb tcpip 5555`.
  O dormitório segue no celular, não baixado.
- USB: wifi_sleep_policy=2 e stay_on_while_plugged_in=7 aplicados; adb tcpip 5555 reconectado.
- Baixadas e conferidas byte a byte, depois apagadas do celular (Regra 16): 100303, 101501, 102600, 112439. APK das
  linhas de apoio instalado.
- DORMITÓRIO 112439:
  - 57 fotos, borrão 3,0-4,8 por estação, nenhum laranja (AF adaptativo + borrão relativo resolveram).
  - Porta (frame 41): linha do teto 2,78, câmera 1,57 (igual ao escritório: pé-direito único confirmado).
  - Planta 3,36 x 3,61 (escalada para 2,70). Se for o dormitório 3,00 x 3,20, dá +12%. Pela porta, H-h = 1,21 contra
    1,23 da planta: o erro está nos ângulos dos vincos (q), não na escala vertical.
- ROOT: proposto (desbloqueio pela conta Motorola, apaga o aparelho, anula a garantia); o usuário RECUSOU por ora.
- CORREÇÃO (usuário): a planta do dormitório 112439, 3,36 x 3,61 m, está "PERFEITA". É outro cômodo, não o de
  3,00 x 3,20; não há os 12% de erro. Referências agora:
  - escritório 2,40 x 3,00 (102600 deu 3,13 x 2,34 a 2,80; ~3,02 x 2,26 a 2,70);
  - dormitório 3,36 x 3,61;
  - pé-direito 2,70; a linha do teto pela porta dá 2,77-2,78 nos dois cômodos.
- Usuário: o erro residual das plantas (escritório ~+1-4%) é tolerável; refinamento fino adiado.
- RETIFICAÇÃO (capInspect --rectify), usuário: o frame 3 foi retificado contra uma fração mínima de parede; com canto nítido, desempatar pela parede com maior exibição. Quadro mirando canto (>= 25 graus) gera UMA vista, a da parede com mais arestas das suas linhas (normal A contém as linhas B: support[2] >= support[1]); sem as duas direções, vale a mais frontal. Dormitório 112439: 57 vistas (antes 76); frame 3 -> parede 3, vinco horizontal na parede dominante.
- NIVELAMENTO PELO VINCO (usuário: "as linhas de teto definem uma trajetória linear que precisa participar da
  normalização"; nos frames 5-10 o horizonte flutuava):
  - capInspect --rectify em duas passadas:
    (1) mede em cada vista frontal a reta do vinco do teto (a linha horizontal forte mais alta por coluna; mínimos
        quadrados, 3 rodadas descartando > 4 px), com inclinação e elevação acima do horizonte nivelado;
    (2) relê a sessão e gira o vertical de cada foto: rolagem = atan(inclinação), para o vinco sair plano;
        inclinação = elevação - mediana da parede (só no giro central, todas as fotos do mesmo ponto); a normal gira
        junto.
  - Sem vinco do teto, usa o do piso (a linha mais baixa), com o dobro de pontos (móveis) e mediana própria.
  - Limites: |rolagem| <= 3 graus, linha a >= 3 graus do horizonte.
  - Foto sem linha: interpola as correções das vizinhas medidas da mesma estação (até 3 fotos).
  - Sinais: a rolagem precisou de -1 (a primeira tentativa dobrava a inclinação); a inclinação, +1.
  - Dormitório 112439: inclinação do vinco -0,040 -> +0,001 (frame 7); elevação 35,32 -> 36,48 (mediana 36,52,
    frame 2); 13 fotos medidas + 2 interpoladas (4 e 9). Nos frames 5-9 o vinco e o topo da janela ficam na mesma
    linha.
- PRÓXIMO (ideia do usuário): no piso, os próprios móveis (cama, mala, quadro) servem de referência. O mesmo objeto
  recortado por dois quadros vizinhos dá a rotação relativa, e a correção de um quadro com vinco se propaga em
  corrente. Cuidado: a paralaxe de objetos fora do plano da parede (raio do giro ~0,3 m).
- ELEIÇÃO DE FOTOS (usuário: "várias colisões de enquadramento"; em debug manter as duas, em produção sobrescrever):
  - registro rtElect = 10 (TElectRecord: cômodo, estação, faixa, bin, electedNs, supersededNs pelo carimbo do
    rtImage), gravado pelo worker a cada foto que entra num bin: quando substitui (laranja ou mais nítida) e quando
    perde (refoto não mais nítida);
  - appKeepSuperseded = 1 (debug) mantém tudo. Com 0 (produção), finishProperty chama TSessionWriter::Compact:
    reescreve capture.lrec sem as imagens substituídas, UMA vez por imóvel (compactar por cômodo reescreveria o
    arquivo inteiro a cada cômodo);
  - Compact copia para capture.lrec.tmp, troca só com a cópia completa e reabre com TRecordLogWriter::Append. Limite:
    ftell em LONG, então arquivo > 2 GB precisa revisão;
  - capInspect: varredura prévia dos rtElect, frames.csv com coluna "elected"; o nivelamento do --rectify pula as
    substituídas;
  - manifesto com "elects"; capTest cobre a compactação.
- Usuário: "ainda não vi o filtro de candidatos de portas rodando". Correto: ele está só no capInspect. PRÓXIMO:
  levá-lo ao app.
- FILTRO DE PORTAS NO APP + ESTAÇÃO DE PORTAS (usuário: "deixe o sprite com a suposta porta e, ao encaixar, a
  confirmação é automática"; botão vermelho "Descartar"; "o giroscópio mais o teto dão a posição do usuário"):
  - core/capDoor:
    - doorFrameWall(vr): o vertical medido e a normal da parede dominante (o mesmo desempate por arestas da
      retificação);
    - doorColumnHeadingDeg: o rumo no mundo de uma coluna da vista frontal.
  - worker, em toda foto medida:
    - vista frontal (buffer PdoorBuf de 4 planos), doorDetect e rumo do centro de cada porta;
    - addDoor funde candidatas da MESMA estação a ±8 graus (cDoorMergeDeg), guarda o ponto de onde foram vistas na
      planta (giro = origem, canto = vértice da estação) e acumula a razão vinco/verga;
    - na grade de giro, as portas da estação aparecem como contorno ciano; a barra de status mostra "portas N".
  - Após o último canto:
    - buildPlanDoors projeta cada candidata na planta (raio até a primeira parede do polígono) e funde as
      repetidas a até 0,6 m;
    - com portas, a nova estação skDoor (rpDoor); sem portas, o cômodo fecha.
  - rpDoor:
    - o olho começa no último canto; o sprite (drawDoorAim) mostra a porta esperada (chão à verga, largura 0,8 m)
      na perspectiva da grade, junto com o quadro da câmera;
    - nivelado (±12), mirando (±8) e firme (< 4 graus/s): foto a cada >= 1,5 s;
    - worker: porta a até ±10 graus do centro da imagem = confirmada (verde, razão somada), segue para a próxima;
    - cada foto confirmada reposiciona o olho: a distância até a parede sai do vinco na vista frontal
      ((vinco - câmera)/tan(e), ambos em vãos de porta), recuada ao longo do raio;
    - "Descartar" (vermelho) marca state 2 e segue; o mapa mostra as portas (branca a fotografar, verde
      confirmada, anel amarelo na atual) e o olho mirando a atual.
  - A meta dos quadros agora tira rumo e inclinação da própria pose (a estação de portas não tem rastreador).
  - PENDENTE: gravar as portas confirmadas num registro próprio; usar a razão confirmada como escala da planta.
- APK instalado.
- ITENS REMANESCENTES DAS PORTAS (usuário: "complete esses dois itens e eu disparo novas capturas"):
  - registro rtDoor = 11 (TDoorRecord): índice, estado dsPending/dsConfirmed/dsDropped, (u, w) na planta na escala
    final, razão vinco/verga média, quadros, carimbo da foto que confirmou. Gravado ao fim da estação de portas
    (finishDoors), também quando o cômodo termina no meio dela. Manifesto com "doors"; capInspect lista cada porta
    e a estação "doors";
  - PORTA COMO RÉGUA: finishDoors soma as razões das portas confirmadas no IMÓVEL e faz
    PpropCeilingM = 2,10 x média. A planta do cômodo é reescalada (layoutScalePlan, agora pública no capLayout e
    usada também pelo Solve) e gravada de novo com lfDoorScaled; portas e olho escalam junto;
  - cômodos seguintes: solvePlan usa PpropCeilingM como hipótese e reescala depois do Solve (nenhum palpite de
    verga pelo histograma pode desfazer); reseta a cada imóvel (openSession);
  - capTest: ida e volta do TDoorRecord e reescala 2,80 -> 2,78. 0 falhas; APK instalado.
- CAPTURA 123038 (escritório com a estação de portas), baixada, conferida e apagada do celular:
  - FALSO DENTE: duas paredes do lado u+, cada uma vista por UMA foto (1,25 m pela foto 0, rasante, com elevação
    11,7 graus e o vinco perto da borda; 1,35 m pela foto 2), mesma parede lisa com moldura contínua. Passaram pela
    exceção "vinco perpendicular termina nela", que num retângulo vale para toda parede perto de canto. CORREÇÃO
    (capLayout): antes das exceções, uma parede secundária com menos de 2 vistas, a menos de cShallowStepM (0,25 m)
    de outra do mesmo lado, é fundida nela (offset pesado, trecho unido, vistas somadas). As exceções ficam para
    degraus fundos (braços de L). 123038: 4 cantos, 2,45 x 3,19; 112439 e 102600 inalterados; capTest ok.
  - PORTAS "na parede oposta": as candidatas (fotos 47 = a folha aberta tomada como porta; 55 = o vão) caíram perto
    do canto 5, justamente o último canto, de onde a fase de portas supunha o operador. Olho em cima da porta: rumo
    sem sentido. CORREÇÃO: a fase de portas começa com o olho no PONTO DO GIRO (a cruz branca); dica "Na cruz
    branca, encaixe a porta amarela do chão à moldura (n)". Cada porta confirmada segue refinando a posição.
  - PENDENTE: a folha aberta detectada como vão (painel liso, com maçaneta, fora do plano da parede) não pode dar
    razão; distinguir pelo interior (o vão mostra o cômodo vizinho).
- APK instalado.
- LAVABO (~1 m, usuário): o piso só aparece com o celular inclinado até -60. TSpinConfig ganhou reachDownDeg/reachUpDeg: a faixa mais baixa aceita até -60 e a mais alta até +60 (ForFov); na vista do piso dos cantos, até -60 (ForFloorView). O leque nivelado dos cantos não muda. capTest: a vista do piso a -58 entra, a -63 fica vermelha. APK instalado.
- CAPTURA 124744 (escritório; conferida e apagada do celular). Porta FALSO POSITIVO na janela (frame 1) e FALSO
  NEGATIVO na porta verdadeira (frames 37, 38, 44). Correções no capDoor:
  - A porta precisa mostrar o PÉ: logo abaixo do batente a vista ainda cobre a foto. Pé cortado pela borda da foto
    só vale se a foto chega a >= 30 graus abaixo do horizonte (porta perto, foto nivelada); aí é só candidata (sem
    razões, altura do vão pela largura x 2,6). A janela (faixa do teto, ~-20 graus) sai.
  - Câmera mínima 0,55 vão (~1,15 m); a janela dava 0,98 m.
  - Porta na parede SECUNDÁRIA dos quadros de canto: doorFrameWalls devolve as duas paredes. O app e o capInspect
    procuram portas nas duas (a retificação segue gravando só a dominante). Cada porta guarda o horizonte, a focal
    e a normal da própria vista (doorShot).
  - Vista frontal com 1600 linhas (antes 1200: o pé caía fora).
  - Busca da verga entre os dois topos (a folha aberta sobe acima dela).
  - Busca do vinco a >= 0,12 vão acima da verga (o topo do alizar, ~0,03, virava "vinco" e esvaziava a caixa de
    cima).
  - Sobel só com os 8 vizinhos válidos (a borda da área coberta criava arestas).
  - Arestas acima da verga: limite 0,10 (parede lisa real: 0,07).
  - Diagnóstico: TDoorStats lista cada par e o motivo com o valor medido (capInspect --doors-all grava toda vista).
  - 124744: janela rejeitada; porta achada no frame 37 (vão na parede 2, com maçaneta e canto; a folha na parede 1).
    Os frames 38 e 44 seguem sem: a porta na borda da vista, sem o outro batente.
- Planta 124744: 3,25 x 4,23 (?), altura da câmera 1,18 (suspeita).
- PRÓXIMO (usuário): WIREFRAME das paredes projetado na prévia (pose + intrínseca + planta: teto a H, piso a 0,
  arestas dos cantos). Cantos do giro do teto já visíveis no giro do piso, mesmo sem a linha do piso (móveis).
- APK instalado.
- WIREFRAME DAS PAREDES NA PRÉVIA (usuário: "a confirmação em wireframe das paredes do ambiente"; "os cantos do
  primeiro giro aparecem no segundo mesmo sem o piso confirmar"; "o giroscópio te ajuda nisso"):
  - drawWireframe logo após a prévia no OnPaint. Por parede: linha do teto (H - h) e do piso (-h); aresta vertical
    em cada canto.
  - Modelo: Pplan quando válida (polígono). Durante o giro central, PguidePlan: as paredes do teto até ali, com os
    cantos de updateGuides (PguideAt), que continuam desenhados no giro do piso.
  - Ponto de vista: centro = ponto do giro; canto = vértice da estação 0,4 m para dentro, rumo ao alvo; portas =
    PdoorEye.
  - Projeção: pose do giroscópio (câmera->mundo, transposta), pinhole da intrínseca NATIVA, inverso do giro de
    updatePreview, escala "cobrir" do canvasBlit (wireToScreen); corte no plano próximo de 5 cm (wireEdge).
  - Alturas: plan.ceilingM e plan.cameraHeightM (1,5 m até haver medida).
  - Não testado em campo.
- APK instalado.
- 132058 (conferida e apagada do celular): wireframe aprovado pelo usuário ("genial"; "percorria o chão com os
  móveis e ele desenhava quase precisamente a linha imaginária do piso").
  - LINHAS DE APOIO REMOVIDAS da grade ("mantenha apenas o wireframe"); updateGuides segue alimentando o wireframe.
  - Porta confirmada SEM razão: de perto a porta inteira com a moldura não cabe numa foto nivelada (foto 53: pé
    sim, moldura fora; fotos 37/38: moldura sim, pé cortado). RAZÃO POR ÂNGULOS: na vista frontal nivelada, todo
    ponto da parede está à mesma distância; altura acima do piso = D (tan e - tan e_pé). TPlanDoor acumula
    tan(verga), tan(pé), tan(vinco) das fotos centradas; com os três,
    razão = (tan e_vinco - tan e_pé)/(tan e_verga - tan e_pé).
  - A porta fica confirmada (verde) na primeira foto e continua a atual até ter pé e moldura. Dicas "incline para
    baixo até o pé" / "incline para cima até a moldura"; inclinação liberada até ±35 na fase de portas; o botão
    vira "Pular medida" (azul). TDoor.footCut é exposto.
  - FOLHA ABERTA como porta (a folha a 90 graus fica paralela à parede vizinha: na vista dela parece uma porta).
    Regra do ALIZAR: uma porta, aberta ou fechada, tem o topo do alizar, uma segunda linha horizontal a
    0,012-0,12 vão acima da verga, atravessando o vão; acima da folha solta só há parede. (A regra do rodapé ao
    lado foi DESCARTADA: sem parede ao lado da folha ela não vê nada.) 132058: o par folha+vão (101-626) é
    barrado; o vão (306-604) segue; as fotos 37/38/53 continuam com o vão.
- APK instalado.
- VISÃO (usuário): metadado por foto -> filtro de móveis (imóvel cru) -> filme -> camadas de mobiliário (do simples ao luxuoso). Registrado em slices/projeto/missao.md.
