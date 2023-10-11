function save_emo_eval(id_pareto, num_ind, storage_pos) {
    return new Promise((resolve, reject) => {
        // id del ind
        let id_ind = `ind_${num_ind}`;

        //se cargan los datos del individuo
        get_dir().then((dir_front) => {
            let dir_histo = `${dir_front}/history`;
            load_object(`${dir_histo}/emotion_evaluation.txt`).then((histo) => {

                if (histo[id_pareto] == undefined) histo[id_pareto] = {};
                if (histo[id_pareto][id_ind] == undefined) histo[id_pareto][id_ind] = {};

                //valores de la métrica emocional
                let valence = parseFloat(document.getElementById("valence").value);
                let arousal = parseFloat(document.getElementById("arousal").value);

                // // se extraen los valores del ind
                // let ind = histo[id_pareto][id_ind][storage_pos];
                // if (ind == undefined) {
                //     console.error("No se encuentra el registro del experimento, id_pareto:", id_pareto, " num_ind:", id_ind, " pos:", storage_pos);
                //     alert("No se encuentra el registro del experimento, id_pareto:", id_pareto, " num_ind:", id_ind, " pos:", storage_pos);
                //     resolve();
                //     return;
                // }
                // registro de la evaluación
                // ind["emotion_evaluation"] = { valence: valence, arousal: arousal };
                histo[id_pareto][id_ind][storage_pos] = { valence: valence, arousal: arousal };


                // se actualiza el historial
                save_object(histo, "emotion_evaluation.txt", dir_histo).then(() => {
                    // se envía la emoción si aplica
                    if (EMOTIONS_CAPTURE_MODE != "sam") {
                        resolve();
                        return;
                    }

                    // se envía la emoción
                    get_actual_user().then((user_name) => {
                        let pub = rosnodejs.nh.advertise("/emotional_prediction", "emotion_classification/emotional_prediction");
                        let msg = { user_name: user_name };
                        msg["id_experiment"] = id_pareto;
                        msg["num_ind"] = num_ind;
                        msg["storage_pos"] = storage_pos;
                        msg["emotion_value"] = [valence, arousal];

                        pub.publish(msg);
                        resolve();
                    }).catch((e) => {
                        alert("No se pudo enviar la emoción al módulo de preferencias");
                        reject(e);
                    });
                }).catch((e) => {
                    reject(e);
                });

            });
        }).catch((e) => {
            reject(e);
        });
    });
}

function show_emo_slider(id_pareto, num_ind, storage_pos, random = false) {
    return new Promise((resolve, reject) => {
        if (id_pareto == "" || num_ind < 0 || storage_pos < 0) {
            reject("Error: valores inválidos para la evaluación");
        }

        //se muestra el slider emocional
        let container_slider = document.getElementById("left_metric");
        container_slider.style.visibility = "visible";
        show_blink(container_slider, color_success);

        //se bloquea la gráfica de los frentes
        document.getElementById("window_block").style.visibility = "visible";
        // se obtiene el botón
        let btn_emo_slider = document.getElementById("russell_point");

        // función de término
        let f_end = function () {
            // se oculta el SAM
            container_slider.style.visibility = "hidden";
            //se desbloquea la gráfica de los frentes
            document.getElementById("window_block").style.visibility = "hidden";

            save_emo_eval(id_pareto, num_ind, storage_pos).then(() => {
                let event = new Event("save_emotion", { bubbles: true });
                btn_emo_slider.dispatchEvent(event);

                resolve();
            }).catch((e) => {
                reject(e);
            });
        }


        if (random) {
            // evaluación ALEATORIA
            document.getElementById("arousal").value = Math.random();
            document.getElementById("valence").value = Math.random();

            btn_emo_slider.onclick = undefined;

            f_end();
        } else {
            // reseteo de los valores
            document.getElementById("arousal").value = 0.5;
            document.getElementById("valence").value = 0.5;

            // evento de la metrica emocional
            btn_emo_slider.onclick = () => {
                f_end();
            }
        }
    });
}
