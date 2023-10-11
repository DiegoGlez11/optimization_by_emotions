



// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// +++++++ evento al dar click al slider de fatiga +++++++++
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++

function save_fatigue_eval(id_pareto, storage_pos, random = false) {
    return new Promise((resolve, reject) => {
        let value = parseFloat(document.getElementById("fatigue_slider").value);

        // ruta del usuario
        get_dir().then((dir_user) => {

            // se guarda el valor de fatiga
            load_object(`${dir_user}/history/fatigue.txt`).then((data_exp) => {
                if (data_exp[id_pareto] == undefined)
                    data_exp[id_pareto] = {};

                data_exp[id_pareto][storage_pos] = { value: value };

                // guardado del valor de fatiga
                save_object(data_exp, "fatigue.txt", `${dir_user}/history`).then(() => {

                    if (value < 0.5 && !random) {
                        //se muestra la ventana si se desea terminar el experimento
                        // document.getElementById("container_fatigue").style.height = "100%";
                        // document.getElementById("container_fatigue").style.visibility = "visible";
                        show_one_window("fatigue");

                        document.getElementById("btn_no").onclick = () => {
                            show_one_window("graph");
                            resolve();
                        };

                    } else {
                        resolve();
                    }


                }).catch(() => {
                    console.error("Error al guardar la fatiga");
                    reject();
                });
            });

        }).catch(() => {
            console.error("Error al obtener el usuario de la fatiga");
            reject();
        });
    });
}

function show_fatigue_slider(id_pareto, storage_pos, random = false) {
    return new Promise((resolve, reject) => {
        if (id_pareto == "" || storage_pos < 0) {
            reject("Error: valores inválidos para la fatiga");
            return;
        }
        DATA_CONTROL.active_fatigue = true;

        // se muestra el slider de fatiga
        let container_slider_fatigue = document.getElementById("right_metric");
        container_slider_fatigue.style.visibility = "visible";
        show_blink(container_slider_fatigue, color_success);


        let f_end = function () {
            container_slider_fatigue.style.visibility = "hidden";

            save_fatigue_eval(id_pareto, storage_pos, random).then(() => {
                DATA_CONTROL.active_fatigue = false;
                resolve();
            }).catch((e) => {
                DATA_CONTROL.active_fatigue = false;
                console.error(e);
                reject();
            });
        }

        let btn_slider = document.getElementById("btn_send_fatigue");
        if (random) {
            // evaluación ALEATORIA
            document.getElementById("fatigue_slider").value = Math.random();

            btn_slider.onclick = undefined;

            f_end();
        } else {
            //reseteo del valor del slider
            document.getElementById("fatigue_slider").value = 0.5;
            btn_slider.onclick = () => {
                f_end();
            }
        }
    });
}


