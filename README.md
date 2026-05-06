# Project: Aquatronic
## Instalación

Es recomendable clonar el proyecto con github

``
git clone https://github.com/PeraltaFede/Aquatronic.git
``

De todas formas, es posible descargar el proyecto como un [.zip](https://github.com/PeraltaFede/Aquatronic/archive/refs/heads/master.zip).

## Uso
Abrir el proyecto: Aquatronic.prj en Matlab.

El projecto está diseñado bajo el paradigma de Programación Orientada a Objetos. El ASV es la clase principal la cual está *compuesta* por uno de los dos observadores actualmente desarrollados (LiuFiniteConvergenceObserver y DualZonotopicObserver). La clase ASV *agrega* las clases de control de Alto y Bajo Nivel HighLevelLOSController y LowLevelAdaptiveController. La clase de alto nivel tiene un equivalente en clase de HighLevelFollowController. El ASV *agrega* corrientes oceánicas a través de la clase OceanCurrents. El [diagrama de clases](https://github.com/PeraltaFede/Aquatronic/blob/Observers/Documentation/Aquatronic%20UML.pdf) está disponible en el proyecto dentro del directorio de Documentación.

### Ejecución de la simulación:
Ejecutar el archivo main_MASV_currents_wind.m

Los historiales completos de `[eta_r nu_r w eta_est nu_est sigma_est]` de todos los ASV se guardan en el elemento `hist_X_0`.
El cual guarda en el mismo vector de forma ordenada, i.e., `X_O = [X_1, X_2, ... , X_n]` siendo `X_i` un vector de 16 elementos que corresponde a los elementos mencionados anterioremente.

## Desarollo

* Existen dos clases Controladores de Alto Nivel (Uno para Virtual target following y otro para real target folowing).
* Existe una clase de Controlador de Bajo Nivel.
* OceanCurrents es la clase que maneja las corrientes oceánicas.
* Las perturbaciones del viento se calculan dentro de la clase ASV.
* De momento, no está implementada la perturbación por olas.
