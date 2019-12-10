from network import network
from Data_processing import Data_processing

    
def pipeline():
    batch_size = 128
    epoch = 3
    model = network()
    data_obj = Data_processing(base_path='./data',batch_size=batch_size)
    data_obj.import_data()
    data_obj.data_sets()
    model.fit_generator(generator = data_obj.train_generator(),
                        validation_data=data_obj.valid_generator(),
                        epochs = epoch,
                        steps_per_epoch = len(data_obj.train)*10 // batch_size,
                        validation_steps = len(data_obj.valid) // batch_size)
    model.save('model.h5')
    return None
    
if __name__ == "__main__":
    pipeline()