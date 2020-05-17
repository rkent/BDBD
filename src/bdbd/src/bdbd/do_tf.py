# uses of tensorflow, without GPU

def classify_face(embedding, svm_model, svm_encoder):
    yhat_trains = svm_model.predict([embedding])
    yhat_probs = svm_model.predict_proba([embedding])
    yhat_names = svm_encoder.inverse_transform(yhat_trains)
    index = yhat_trains[0]
    prob = round(yhat_probs[0][index] * 100., 1)

    return (yhat_names[0], prob)

def do_tf():
    try:
        import logging
        import logging.handlers
        import traceback
        from queue import Queue
        queue = Queue()

        #import configuration
        #config = configuration.get()
        log_level = 'INFO'
        log_format = '%(name)-20s%(process)-8s%(levelname)-9s%(asctime)s: %(message)s'

        logging.basicConfig(level=log_level, format=log_format)
        log = logging.getLogger('do_tf')

        from bdbd.libpy.MessageClient import MessageClient
        mc = MessageClient('localhost', 'beedee-do_tf', queue)
        mc.connect()
        mc.subscribe('beedee/command')
        mc.subscribe('beedee/get_facename', msgType='bytes')

        import os
        log.info(f'do_tf starting with PID {str(os.getpid())}')

        os.environ["CUDA_VISIBLE_DEVICES"]="-1"
        import cv2
        import numpy as np
        import time
        import pickle
        from libpy.handleErrors import Errors

        from libpy.get_embedding import get_embedding
        log.info('Start loading tf')
        start = time.time()
        import tensorflow as tf
        keras = tf.keras
        log.info(f'Done loading tf, elapsed time {round(time.time() - start, 3)} seconds')

        from tensorflow.keras.models import load_model
        log.info('Loading embedding model')
        start = time.time()
        file_dir = os.path.dirname(os.path.abspath(__file__))
        model = load_model(file_dir + '/models/facenet_keras.h5')

        data_dir = '/bddata/'
        (svm_model, svm_encoder) = pickle.load(open(file_dir + data_dir + 'family_face_svm.p', 'rb'))

        log.info(f'embedding model loaded, elapsed time {round(time.time() - start, 3)} seconds')

        # prime the pump by evaluating a single image
        # load the face dataset
        data = np.load(file_dir + data_dir + 'faces.npz', allow_pickle=True)
        X = data['arr_0']
        start = time.time()
        log.info('Starting initial embedding calculation to prime the pump')
        embedding = get_embedding(model, X[0])
        log.info(f'Got embedding, elapsed time {round(time.time() - start, 3)}')
        name, prob = classify_face(embedding, svm_model, svm_encoder)
        log.info('classified as name: {} probability: {:6.3f}'.format(name, prob))
 
        errors = Errors(log)
        log.info('do_tf entering main loop')        
        mc.publish('beedee/command', 'GET_EMBEDDING.READY')
        mc.publish('beedee/sayit', 'Tensor flow ready for names')
        while True:
            try:
                topic, message = queue.get()
                if topic == 'beedee/command':
                    if message == '_quit' or message == 'do_tf_quit':
                        break

                if topic == 'beedee/get_facename':
                    frame_jpg = np.asarray(bytearray(message), dtype='uint8')
                    frame = cv2.imdecode(frame_jpg, cv2.IMREAD_COLOR)
                    log.info(f'get_facename request')
                    embedding = get_embedding(model, frame)
                    name, prob = classify_face(embedding, svm_model, svm_encoder)
                    log.info('publishing face name: {} prob: {:6.3f}'.format(name, prob))
                    mc.publish('beedee/facename', {'name': name, 'probability': prob}, msgType='json')

            except KeyboardInterrupt:
                break
            except:
                if errors.handle():
                    break
                else:
                    log.info(f'do_tf ignoring error, continuing')

    except:
        exc = traceback.format_exc()
        log.error(f'Exception caught in do_tf:\n{exc}')
    finally:
        mc.close()
        tf.keras.backend.clear_session()
        log.info(f'do_tf ending')
    return 0

do_tf()
