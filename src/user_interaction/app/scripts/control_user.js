



/*
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
Cada vez que se actualiza el explorador se cargan los frentes de pareto
que ya se habían hecho con anterioridad
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
*/

function get_actual_user() {
    return new Promise((resolve, reject) => {
        let srv = rosnodejs.nh.serviceClient("/get_user", "experiment_control_interfaces/get_user");

        srv.call({}).then((res) => {
            if (res.user_name == "") {
                alert(`Error al obtener el usuario. No hay usuario`);
                reject(alert(`Error al obtener el usuario`));
                return;
            }
            resolve(res.user_name);
        }).catch((e) => {
            if (is_conn_refused(e)) e = "";
            alert(`Error al obtener el usuario`);
            reject(`Error al obtener el usuario\n\n${e}`);
        });
    });
}


// ---devuelve la ruta deseada
function get_dir(id_pareto = undefined, num_ind = undefined, type_front = false) {
    return new Promise((resolve, reject) => {
        get_actual_user().then((user_name) => {
            let dir_user = `${root_dir}/${user_name}`;

            if (id_pareto == undefined) resolve(dir_user);
            else {
                //typo del id
                get_type_id(id_pareto).then((type_name) => {
                    let dir_type_front = `${dir_user}/${type_name}`;

                    if (type_front) {
                        resolve(dir_type_front);
                    } else {
                        let dir_front = `${dir_type_front}/${id_pareto}`;
                        //número de individuo
                        if (num_ind != undefined) dir_front = `${dir_front}/ind_${num_ind}`;

                        resolve(dir_front);
                    }
                }).catch((e) => {
                    reject(e);
                });
            }
        }).catch((e) => {
            reject(e);
        });
    });
}



