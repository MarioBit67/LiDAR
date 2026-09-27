# Regra 8. Host Android em C++ puro (NativeActivity)

O host Android é NativeActivity em C++ puro: câmera NDK (Camera2), ASensor, desenho no ANativeWindow e JNI
chamado a partir do C++. Sem Kotlin/Java. Consequência aceita: o GPS vem de
`LocationManager.getLastKnownLocation` consultado periodicamente (sem listener contínuo); as permissões são
verificadas por polling.

Origem (usuário, 2026-09-26): escolheu "C++ puro (NativeActivity)" em vez de um host Kotlin fino.
