# LiDAR - Regras de Ouro

Regras operacionais do projeto de captura LiDAR (iOS + Android, última milha). Cada regra nasce de uma
diretiva do usuário neste projeto; nenhuma é importada de outro projeto.

> ⚠️ **REGRA 0 - RELEIA ANTES DE QUALQUER PASSO GRANDE.** Antes de uma mudança de arquitetura (formato de
> sessão, porta de plataforma, pipeline de captura), pare e releia este índice e as fatias relevantes.

> 📂 **Layout:** o texto completo de cada regra fica em `slices/regraOuro/NN-slug.md`. Este índice é o
> checklist vinculante; os títulos SÃO as regras em resumo. Regra nova = fatia nova + uma linha aqui.

## Regras

- [1. NADA de Python - nunca](slices/regraOuro/01-nada-de-python.md)
- [2. C++20 puro, código denso proibido](slices/regraOuro/02-cpp20-puro-sem-codigo-denso.md)
- [3. Toda compilação passa pelo ppCompile original; ppCheck -deep antes de qualquer build](slices/regraOuro/03-ppcompile-obrigatorio.md)
- [4. E:\Projetos\Claude nunca é alterado; shared/ é replicável, o resto só com consentimento](slices/regraOuro/04-monorepo-claude-somente-leitura.md)
- [5. memorias/ é o repositório oficial - replica para o pessoal, nunca o contrário](slices/regraOuro/05-memorias-oficiais.md)
- [6. Código sempre comum às plataformas - adaptadores finos atrás de uma porta abstrata](slices/regraOuro/06-codigo-comum-porta-abstrata.md)
- [7. Android é o foco; iOS aguarda a cortesia do agente Mac](slices/regraOuro/07-android-primeiro.md)
- [8. Host Android em C++ puro (NativeActivity)](slices/regraOuro/08-host-android-cpp-puro.md)
- [9. Cada frame guardado leva orientação absoluta (bússola) - o cômodo nasce alinhado ao norte](slices/regraOuro/09-frame-com-bussola.md)
- [10. Todo agente novo lê as memórias e confirma verbalmente antes de agir](slices/regraOuro/10-onboarding-com-confirmacao.md)
- [11. Geometria é a essência do coletor - GPS + bússola primeiro, linhas de piso e teto completam](slices/regraOuro/11-geometria-e-a-essencia.md)
- [12. Sem pedidos de confirmação - decida, registre no jobLog e siga](slices/regraOuro/12-sem-confirmacoes.md)
- [13. Protocolo do cômodo - dois giros centrais, depois os cantos que a planta indica (adaptativo)](slices/regraOuro/13-protocolo-do-comodo.md)
- [14. Espelhar as memórias só no fim da sessão (ou quando pedido)](slices/regraOuro/14-espelho-no-fim.md)
- [15. Ordem do TODO: em operação no topo, pendentes no meio, resolvidos no final](slices/regraOuro/15-ordem-do-todo.md)
- [16. Baixou a captura (conferida), apaga do celular](slices/regraOuro/16-baixou-apaga-do-celular.md)
