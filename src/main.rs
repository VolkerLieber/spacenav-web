use std::{collections::HashMap, string};

use futures_util::{
    stream::{SplitSink, SplitStream},
    FutureExt, SinkExt, StreamExt,
};
use warp::{
    ws::{Message, WebSocket},
    Filter,
};

use rand::distributions::Alphanumeric;
use rand::{thread_rng, Rng};
use serde_json::{self, Number, Value};

#[tokio::main]
async fn main() {
    let websocket = warp::path::end()
        .and(warp::ws())
        .map(|ws: warp::ws::Ws| {
            // This will call our function if the handshake succeeds.
            ws.on_upgrade(move |socket| {
                // let (tx, rx) = socket.split();
                // // rx.forward(tx).map(|result| {
                // //     if let Err(e) = result {
                // //         eprintln!("websocket error: {:?}", e);
                // //     }
                // // })
                // send_welcome(&tx);

                // rx.forward(sink)
                handle_session(socket)
            })
        })
        .with(warp::reply::with::header("Sec-WebSocket-Protocol", "wamp"));

    // GET / -> index html
    let proxy = warp::path!("3dconnexion" / "nlproxy").map(|| "{\"port\":8181}");

    let routes = proxy.or(websocket).with(warp::reply::with::header(
        "Access-Control-Allow-Origin",
        "*",
    ));

    warp::serve(routes)
        .tls()
        .cert_path("C:\\Program Files (x86)\\3Dconnexion\\3DxWare\\3DxNLServer\\bin\\server.crt")
        .key_path("C:\\Program Files (x86)\\3Dconnexion\\3DxWare\\3DxNLServer\\bin\\server.key")
        .run(([127, 51, 68, 120], 8181))
        .await;
}

async fn send_welcome(tx: &mut SplitSink<WebSocket, Message>) {
    tx.send(Message::text(format!("[0,{:?},1,\"Nl-Proxy v1.4.3.19386 Copyright 2013-2022 3Dconnexion. All rights reserved.\"]", "8GXm6SS4smp3Ai0e"))).await.unwrap();
}

enum ClientReturnHandlers {
    SelectionEmpty,
    ViewPerspective,
    ViewAffine,
    ViewTarget,
}

type Vector = [f32; 3];
trait VectorOperationable {
    fn cross(&self, vb: &Vector) -> [f32; 3];
    fn qrot(&mut self, quat: &Quat);
}

impl VectorOperationable for Vector {
    fn cross(&self, vb: &Vector) -> [f32; 3] {
        [
            self[1] * vb[2] - self[2] * vb[1],
            self[2] * vb[0] - self[0] * vb[2],
            self[0] * vb[1] - self[1] * vb[0],
        ]
    }

    fn qrot(&mut self, quat: &Quat) {
        let mut inv_q: Quat = quat.clone();
        let mut tmp_q: Quat = quat.clone();
        let vq: Quat = [self[0], self[1], self[2], 0.0];

        inv_q.invert();
        tmp_q.mult(&vq);
        tmp_q.mult(&inv_q);

        self[0] = tmp_q[0];
        self[1] = tmp_q[1];
        self[2] = tmp_q[2];
    }
}

type Quat = [f32; 4];
trait QuatOperationable {
    fn rotate(&mut self, angle: f32, x: f32, y: f32, z: f32);
    fn vec3_cross(&self, vb: &Quat) -> [f32; 3];
    fn mult(&mut self, qb: &Quat);
    fn invert(&mut self);
}

impl QuatOperationable for Quat {
    fn rotate(&mut self, angle: f32, x: f32, y: f32, z: f32) {
        let mut rq: Quat = [0.0; 4];
        let half = angle * 0.5;
        let sin_half = half.sin();

        rq[3] = half.cos();
        rq[0] = x * sin_half;
        rq[1] = y * sin_half;
        rq[2] = z * sin_half;

        self.mult(&rq);
    }

    fn vec3_cross(&self, vb: &Quat) -> [f32; 3] {
        let a: &[f32; 3] = self[..4].try_into().unwrap();
        let b: &[f32; 3] = vb[..4].try_into().unwrap();
        a.cross(b)
    }

    fn mult(&mut self, qb: &Quat) {
        let dot = self[0] * qb[0] + self[1] * qb[1] * self[2] * qb[2];
        let cross = qb.vec3_cross(self);

        let x = self[3] * qb[0] + qb[3] * self[0] + cross[0];
        let y = self[3] * qb[1] + qb[3] * self[1] + cross[1];
        let z = self[3] * qb[2] + qb[3] * self[2] + cross[2];

        self[3] = self[3] * qb[3] - dot;
        self[0] = x;
        self[1] = y;
        self[2] = z;
    }

    fn invert(&mut self) {
        let len_sq = self[0] * self[0] + self[1] * self[1] + self[2] * self[2] + self[3] * self[3];

        self[0] = -self[0];
        self[1] = -self[1];
        self[2] = -self[2];

        if len_sq < 0.0 || len_sq > 0.0 {
            let s = 1.0 / len_sq;
            self[0] *= s;
            self[1] *= s;
            self[2] *= s;
            self[3] *= s;
        }
    }
}

struct spnav_event_motion {
    event_type: i32,
    x: i32,
    y: i32,
    z: i32,
    rx: i32,
    ry: i32,
    rz: i32,
    period: u32,
    // int *data;
}

struct spnav_posrot {
    pos: [f32; 3],
    rot: Quat,
}
impl spnav_posrot {
    pub fn new() -> spnav_posrot {
        spnav_posrot {
            pos: [0.0, 0.0, 0.0],
            rot: [0.0, 0.0, 0.0, 1.0],
        }
    }

    /*
        static void quat_invert(float *q)
    {
        float s, len_sq = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
        /* conjugate */
        q[0] = -q[0];
        q[1] = -q[1];
        q[2] = -q[2];
        if(len_sq != 0.0f) {
            s = 1.0f / len_sq;
            q[0] *= s;
            q[1] *= s;
            q[2] *= s;
            q[3] *= s;
        }
    }
     */

    /*
        static void vec3_qrot(float *res, const float *vec, const float *quat)
    {
        float vq[4], inv_q[4], tmp_q[4];

        inv_q[0] = tmp_q[0] = quat[0];
        inv_q[1] = tmp_q[1] = quat[1];
        inv_q[2] = tmp_q[2] = quat[2];
        inv_q[3] = tmp_q[3] = quat[3];

        vq[0] = vec[0];
        vq[1] = vec[1];
        vq[2] = vec[2];
        vq[3] = 0.0f;

        quat_invert(inv_q);
        quat_mul(tmp_q, vq);
        quat_mul(tmp_q, inv_q);

        res[0] = tmp_q[0];
        res[1] = tmp_q[1];
        res[2] = tmp_q[2];
    }
    */

    pub fn move_view(&mut self, motion: &spnav_event_motion) {
        let len: f32 =
            ((motion.rx * motion.rx + motion.ry * motion.ry + motion.rz * motion.rz) as f32).sqrt();

        if len < 0.0 || len > 0.0 {
            let x = (-motion.rx as f32) / len;
            let y = (-motion.ry as f32) / len;
            let z = (-motion.rz as f32) / len;
            self.rot.rotate(len * 0.001, x, y, z);
        }

        let mut trans: [f32; 3] = [
            (-motion.x as f32) * 0.001,
            (-motion.y as f32) * 0.001,
            (motion.z as f32) * 0.001,
        ];

        trans.qrot(&self.rot);

        self.pos[0] += trans[0];
        self.pos[1] += trans[1];
        self.pos[2] += trans[2];
    }
}

struct Session {
    transmitter: SplitSink<WebSocket, Message>,
    receiver: SplitStream<WebSocket>,
    instance: u32,
    callbacks: HashMap<String, ClientReturnHandlers>,
    position: spnav_posrot,
    view_matrix: [f32; 16],
}
impl Session {
    fn new(socket: WebSocket) -> Session {
        let (session_tx, session_rx) = socket.split();
        Session {
            transmitter: session_tx,
            receiver: session_rx,
            instance: thread_rng().gen(),
            callbacks: HashMap::new(),
            position: spnav_posrot::new(),
            view_matrix: [0.0; 16],
        }
    }
}

async fn handle_session(socket: WebSocket) {
    println!("NEW SESSION");

    let mut session = Session::new(socket);

    send_welcome(&mut session.transmitter).await;

    while let Some(result) = session.receiver.next().await {
        let msg = match result {
            Ok(msg) => msg,
            Err(e) => {
                println!("WS ERROR: {e}");
                break;
            }
        };
        handle_msg(msg, &mut session).await;
    }
}

fn parse_msg<'a>(msg: Message) -> Result<(MessageType, Value), ()> {
    // Parse as json
    let json: Value = serde_json::from_slice(msg.as_bytes()).map_err(|err| ())?;

    // Expect array
    let json_array = json.as_array().ok_or(())?;

    // Expect length
    match json_array.len() {
        2..=6 => (),
        _ => return Err(()),
    };

    // Parse MessageType
    let msgType: MessageType = match &json_array[0] {
        Value::Number(msgType) => match msgType.as_u64() {
            Some(msgType) if msgType <= 8 => MessageType::from_u64(msgType),
            _ => return Err(()),
        },
        _ => return Err(()),
    };

    Ok((msgType, json))
}

async fn handle_msg(msg: Message, session: &mut Session) {
    println!("MESSAGE: {:?}", msg);

    let (msgType, msg) = match parse_msg(msg) {
        Ok((msgType, json)) => (msgType, json),
        Err(_) => return,
    };

    let json = msg.as_array().expect("Unwrapping is handled in parse_msg");

    let ret = match msgType {
        MessageType::Welcome => return, // Server
        MessageType::Prefix => return,  // Client: Can be ignored
        MessageType::Call => handle_call(json, session),
        MessageType::CallResult => {
            println!("CallResult: {:?}", json);
            let callback_id = match json[1].as_str() {
                Some(callback_id) => callback_id,
                None => return,
            };
            let return_handler = match session.callbacks.remove(callback_id) {
                Some(return_handler) => return_handler,
                None => return,
            };
            match return_handler {
                ClientReturnHandlers::SelectionEmpty => todo!(),
                ClientReturnHandlers::ViewPerspective => todo!(),
                ClientReturnHandlers::ViewAffine => todo!(),
                ClientReturnHandlers::ViewTarget => todo!(),
            };
            return;
        }
        MessageType::CallError => {
            println!("CallError: {:?}", json);
            return;
        }
        MessageType::Subscribe => {
            println!("Subscribe: {:?}", json);

            // Init variables
            // tx.send(build_update_call(
            //     instance,
            //     &generate_id(),
            //     "self:read",
            //     "selection.empty",
            // ))
            // .await;

            // tx.send(build_update_call(
            //     instance,
            //     &generate_id(),
            //     "self:read",
            //     "view.perspective",
            // ))
            // .await;

            // tx.send(build_update_call(
            //     instance,
            //     &generate_id(),
            //     "self:read",
            //     "view.affine",
            // ))
            // .await;

            // tx.send(build_update_call(
            //     instance,
            //     &generate_id(),
            //     "self:read",
            //     "view.target",
            // ))
            // .await;

            // [8,"3dconnexion:3dcontroller/6884113743086",[2,"t3bLavZGeDHtHSly","self:update","","motion",true]]
            session
                .transmitter
                .send(build_update_call(
                    session.instance,
                    &generate_id(),
                    "motion",
                    "true",
                ))
                .await;

            return;
        }
        MessageType::Unsubscribe => {
            println!("Unsubscribe: {:?}", json);
            return;
        }
        MessageType::Publish => {
            println!("Publish: {:?}", json);
            return;
        }
        MessageType::Event => return, // Server
    };

    let success = match ret {
        Ok(ret) => session.transmitter.send(ret).await,
        Err(_) => return,
    };

    match success {
        Ok(_) => return,
        Err(_) => println!("ERROR WHILE SENDING"),
    };
}

/*
[2,"0.wu6w9bnqe4","3dx_rpc:create","3dconnexion:3dmouse","0.6.0"]
[2,"0.h2jd78cvemc","3dx_rpc:update","3dconnexion:3dcontroller/6884113743086",{"commands":{"activeSet":"Part Studio"}}]
[2,"0.pim5f32a7ff","3dx_rpc:update","3dconnexion:3dcontroller/6884113743086",{"focus":true}]
 */
fn handle_call(json: &Vec<Value>, session: &mut Session) -> Result<Message, ()> {
    // if (json.len() !== )

    let msg_id = json[1].as_str().ok_or(())?;

    let fn_name: &str = json[2].as_str().ok_or(())?;

    Ok(match fn_name {
        "3dx_rpc:create" => match json[3].as_str().ok_or(())? {
            "3dconnexion:3dmouse" => {
                build_result(msg_id, format!("{{\"connexion\":\"{}\"}}", generate_id()))
            }
            "3dconnexion:3dcontroller" => {
                build_result(msg_id, format!("{{\"instance\":{}}}", session.instance))
            }
            _ => {
                println!("UNHANDLED CALL: {:?}", json);
                return Err(());
            }
        },
        "3dx_rpc:update" => {
            println!("UPDATE: {:?}", json);
            if let Value::Object(map) = &json[4] {
                if let Some((_, Value::Object(frame))) = map.get_key_value("frame") {
                    if let Some((_, Value::Number(time))) = frame.get_key_value("time") {
                        session.transmitter.send(build_update_call(
                            session.instance,
                            &generate_id(),
                            "transaction",
                            "1",
                        ));

                        session.transmitter.send(build_update_call(
                            session.instance,
                            &generate_id(),
                            "transaction",
                            "1",
                        ));
                    }
                }
            }

            build_result(msg_id, "{}".to_string())
        }
        _ => {
            println!("UNHANDLED CALL: {:?}", json);
            return Err(());
        }
    })
}

/*
{
  "_version": "0.8.2.1",
  "browserNotSupportedMessage": "Browser does not support WebSockets (RFC6455)",
  "_idchars": "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789",
  "_idlen": 16,
  "_subprotocol": "wamp",
  "_debugrpc": false,
  "_debugpubsub": false,
  "_debugws": false,
  "_debugconnect": false,
  "_MESSAGE_TYPEID_WELCOME": 0,
  "_MESSAGE_TYPEID_PREFIX": 1,
  "_MESSAGE_TYPEID_CALL": 2,
  "_MESSAGE_TYPEID_CALL_RESULT": 3,
  "_MESSAGE_TYPEID_CALL_ERROR": 4,
  "_MESSAGE_TYPEID_SUBSCRIBE": 5,
  "_MESSAGE_TYPEID_UNSUBSCRIBE": 6,
  "_MESSAGE_TYPEID_PUBLISH": 7,
  "_MESSAGE_TYPEID_EVENT": 8,
  "CONNECTION_CLOSED": 0,
  "CONNECTION_LOST": 1,
  "CONNECTION_RETRIES_EXCEEDED": 2,
  "CONNECTION_UNREACHABLE": 3,
  "CONNECTION_UNSUPPORTED": 4,
  "CONNECTION_UNREACHABLE_SCHEDULED_RECONNECT": 5,
  "CONNECTION_LOST_SCHEDULED_RECONNECT": 6,
  "_UA_FIREFOX": {},
  "_UA_CHROME": {},
  "_UA_CHROMEFRAME": {},
  "_UA_WEBKIT": {},
  "_UA_WEBOS": {}
}
 */

#[derive(Debug)]
enum MessageType {
    /// 0: Server greets client
    Welcome,
    /// 1: Client greets server
    Prefix,
    /// 2: Client calls server function
    Call,
    /// 3: Return of server/client function
    CallResult,
    /// 4: Client sends event notice to server
    CallError,
    /// 5: Client joined room specified by server
    Subscribe,
    /// 6
    Unsubscribe,
    /// 7
    Publish,
    /// 8: Server calls client function
    Event,
}
impl MessageType {
    fn from_u64(value: u64) -> MessageType {
        match value {
            0 => MessageType::Welcome,
            1 => MessageType::Prefix,
            2 => MessageType::Call,
            3 => MessageType::CallResult,
            4 => MessageType::CallError,
            5 => MessageType::Subscribe,
            6 => MessageType::Unsubscribe,
            7 => MessageType::Publish,
            8 => MessageType::Event,
            _ => {
                println!("UNEXPECTED MESSAGE TYPE: {value}");
                MessageType::Welcome
            }
        }
    }
}

/*
8GXm6SS4smp3Ai0e
 */
fn generate_id() -> String {
    thread_rng()
        .sample_iter(&Alphanumeric)
        .take(16)
        .map(char::from)
        .collect()
}

/*
[0,"8GXm6SS4smp3Ai0e",1,"Nl-Proxy v1.4.3.19386 Copyright 2013-2022 3Dconnexion. All rights reserved."]
 */
fn build_welcome(id: &str) -> Message {
    Message::text(format!(
        "[{},{},1,\"Spacenav Proxy\"]",
        MessageType::Welcome as u32,
        id
    ))
}

/*
[3,"0.pim5f32a7ff",{}]
 */
fn build_result(id: &str, data: String) -> Message {
    Message::text(format!(
        "[{},\"{}\",{}]",
        MessageType::CallResult as u32,
        id,
        data
    ))
}

/*
[8,"3dconnexion:3dcontroller/6884113743086",[2,"1cJSqNoRqbVxr4Ds","self:update","","hit.selectionOnly",false]]
[8,"3dconnexion:3dcontroller/6884113743086",[2,"DMr5ZjiANwTtk65Y","self:update","","settings.changed",2]]
[8,"3dconnexion:3dcontroller/6884113743086",[2,"AEZWTPeAMmbGLPIB","self:read","","coordinateSystem"]]
 */
fn build_read_call(instance: u32, id: &str, key: &str) -> Message {
    Message::text(format!(
        "[{},\"3dconnexion:3dcontroller/{}\",[{},\"{}\",\"{}\",\"\",\"{}\"]]",
        MessageType::Event as u32,
        instance,
        MessageType::Call as u32,
        id,
        "self::read",
        key
    ))
}

fn build_update_call(instance: u32, id: &str, key: &str, value: &str) -> Message {
    Message::text(format!(
        "[{},\"3dconnexion:3dcontroller/{}\",[{},\"{}\",\"{}\",\"\",\"{}\",{}]]",
        MessageType::Event as u32,
        instance,
        MessageType::Call as u32,
        id,
        "self::update",
        key,
        value
    ))
}

/*
[8,"3dconnexion:3dcontroller/6884113743086",[2,"xqys5A4ZiD8E3lla","self:read","","selection.empty"]]
[8,"3dconnexion:3dcontroller/3042851224",[2,"HxMA6bihFAoFhhHK","self::read","","selection.empty"]]
 */
