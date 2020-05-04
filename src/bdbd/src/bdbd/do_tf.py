# uses of tensorflow, without GPU

__name__ = 'do_tf'

def classify_face(embedding, svm_model, svm_encoder):
    yhat_trains = svm_model.predict([embedding])
    yhat_probs = svm_model.predict_proba([embedding])
    yhat_names = svm_encoder.inverse_transform(yhat_trains)
    index = yhat_trains[0]
    prob = round(yhat_probs[0][index] * 100., 1)

    return (yhat_names[0], prob)

def do_tf():
    from queue import Queue
    queue = Queue()
    from functools import partial
    import signal
    def quit(q, sig, frame):
        print(f'handling signal {sig} in {__name__}', flush=True)
        q.put(['beedee/command', '_quit'])
    sigs = [signal.SIGHUP, signal.SIGTERM]
    for s in sigs:
        signal.signal(s, partial(quit, queue))

    try:
        import logging
        import logging.handlers
        import traceback

        #import configuration
        #config = configuration.get()
        log_level = 'INFO'
        log_format = '%(name)-20s%(process)-8s%(levelname)-9s%(asctime)s: %(message)s'

        logging.basicConfig(level=log_level, format=log_format)
        log = logging.getLogger(__name__)

        from bdbd.libpy.MessageClient import MessageClient
        mc = MessageClient('localhost', 'beedee-do_tf', queue)
        mc.connect()
        mc.subscribe('beedee/command')
        mc.subscribe('beedee/get_embedding', msgType='bytes')

        import os
        log.info(f'{__name__} starting with PID {str(os.getpid())}')

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
        model = load_model('models/facenet_keras.h5')

        data_dir = 'bddata/'
        (svm_model, svm_encoder) = pickle.load(open(data_dir + 'family_face_svm.p', 'rb'))

        log.info(f'embedding model loaded, elapsed time {round(time.time() - start, 3)} seconds')

        # prime the pump by evaluating a single image
        # load the face dataset
        data = np.load('bddata/faces.npz', allow_pickle=True)
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
                    if message == '_quit':
                        break

                if topic == 'beedee/get_embedding':
                    id, facePixels = pickle.loads(message)
                    log.info(f'get_embedding request with id {id}')
                    embedding = get_embedding(model, facePixels)
                    name, prob = classify_face(embedding, svm_model, svm_encoder)
                    log.info('publishing face name: {} prob: {:6.3f}'.format(name, prob))
                    mc.publish('beedee/facename', {name: name, probability: prob}, msgType='json')

            except KeyboardInterrupt:
                pass
            except:
                if errors.handle():
                    break
                else:
                    log.info(f'{__name__} ignoring error, continuing')

    except:
        exc = traceback.format_exc()
        log.error(f'Exception caught in {__name__}:\n{exc}')
    finally:
        mc.close()
        tf.keras.backend.clear_session()
        log.info(f'{__name__} ending')
    return 0

do_tf()

# demo as standalone
'''
if __name__ == '__main__':

    # given embedding, classify the face

    import pickle

    data_dir = 'bddata/'
    (svm_model, svm_encoder) = pickle.load(open(data_dir + 'family_face_svm.p', 'rb'))

    def classify_face(embedding):
        yhat_trains = svm_model.predict([embedding])
        yhat_probs = svm_model.predict_proba([embedding])
        yhat_names = svm_encoder.inverse_transform(yhat_trains)
        index = yhat_trains[0]
        prob = round(yhat_probs[0][index] * 100., 1)
    
        return (yhat_names[0], prob)

    import time
    from multiprocessing import Process
    from bdbd.libpy.MessageClient import MessageClient
    import numpy as np
    import pickle
    from queue import Queue
    queue = Queue()

    import logging
    logging.basicConfig(level='DEBUG')
    log = logging.getLogger(__name__)

    p = Process(target=do_tf, args=((None),))
    p.start()
 
    queue=Queue()
    mc = MessageClient('localhost', 'beedee-do_tf_demo', queue)
    mc.connect()
    mc.subscribe('beedee/command')
    mc.subscribe('bddata/put_embedding', msgType='bytes')

    data = np.load('bddata/faces.npz', allow_pickle=True)
    X = data['arr_0']

    # Wait for do_tr to signal ready
    while True:
        topic, message = queue.get()
        if topic == 'beedee/command':
            if message == 'GET_EMBEDDING.READY':
                break

    start = time.time()
    id = 'dummy'
    pickled_image = pickle.dumps((id, X[0]))
    mc.publish('beedee/get_embedding', pickled_image, msgType='bytes')

    while True:
        try:
            topic, message = queue.get()
            if topic == 'bddata/put_embedding':
                id, embedding = pickle.loads(message)
                log.info(f'got embedding with id {id}')
                log.info(f'type of embedding: {type(embedding)}')
                log.info(f'elapsed time: {round(time.time() - start, 3)}')
                name, prob = classify_face(embedding)
                log.info(f'name {name} ({round(prob, 1)}%)')
        except KeyboardInterrupt:
            mc.publish('beedee/command', '_quit')
            break
    log.info('wait for process to accept _quit and end')
    p.join()
    mc.close()
'''