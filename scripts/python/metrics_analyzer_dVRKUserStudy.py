import csv
from fileinput import filename
import os
import numpy as np
import pandas as pd

# pedal_path = '/home/npasini1/Desktop/dVRK_UserStudy/dVRK_metrics/pedal'
# scan_path = '/home/npasini1/Desktop/dVRK_UserStudy/dVRK_metrics/autonomous'
# auto_path = '/home/npasini1/Desktop/dVRK_UserStudy/dVRK_metrics/autonomous'

pedal_path = '/home/npasini1/Desktop/pedal'
scan_path = '/home/npasini1/Desktop/scan'
auto_path = '/home/npasini1/Desktop/autonomous'

pedalFiles = os.listdir(pedal_path)
scanFiles = os.listdir(scan_path)
autoFiles = os.listdir(auto_path)
fileList = []

folderPaths = [pedal_path, scan_path, auto_path]
fileNames = [pedalFiles, scanFiles, autoFiles]

modalities = ['clutch','scan','auto']

for i in range(3):
    times = []
    clutch_pedal_presses = []
    camera_pedal_presses = []
    PSM1_distance_covered = []
    PSM2_distance_covered = []
    ECM_distance_covered = []
    comfort_distance_MTMR = []
    comfort_distance_MTML = []
    modality = modalities[i]

    for fileName in fileNames[i]:
        if fileName.endswith(".csv"):
            fileList.append(fileName)
            path = os.path.join(folderPaths[i], fileName)

            print('Now reading file: ', path)
            dataframe = pd.read_csv(path)

            time = dataframe['Execution time'][0]
            clutch = dataframe['Clutch presses'][0]
            camera = dataframe['Camera presses'][0]
            PSM1 = dataframe['Total PSM1 path length'][0]
            PSM2 = dataframe['Total PSM2 path length'][0]
            MTMR = dataframe['Current distance MTMR'][:-10]
            MTML = dataframe['Current distance MTML'][:-10]
            ECM = pd.DataFrame()
            try:
                ECM = dataframe['Total ECM path length'][0]
            except:
                print('File ' + fileName + ' does not contain ECM distance covered info, sorry :(')

            times.append(time)
            clutch_pedal_presses.append(clutch)
            camera_pedal_presses.append(camera)
            PSM1_distance_covered.append(PSM1)
            PSM2_distance_covered.append(PSM2)
            ECM_distance_covered.append(ECM)
            comfort_distance_MTMR.extend(MTMR)
            comfort_distance_MTML.extend(MTML)

    df_Time = pd.DataFrame(times, columns = ['Time'])
    df_Clutch = pd.DataFrame(clutch_pedal_presses, columns = ['Clutch'])
    df_Camera = pd.DataFrame(camera_pedal_presses, columns = ['Camera'])
    df_PSM1 = pd.DataFrame(PSM1_distance_covered, columns = ['PSM1'])
    df_PSM2 = pd.DataFrame(PSM2_distance_covered, columns = ['PSM2'])
    df_ECM = pd.DataFrame(ECM_distance_covered, columns = ['ECM'])
    df_MTMR = pd.DataFrame(comfort_distance_MTMR, columns = ['MTMR'])
    df_MTML = pd.DataFrame(comfort_distance_MTML, columns = ['MTML'])

    df = pd.concat([df_Time,df_Clutch,df_Camera,df_PSM1,df_PSM2,df_ECM,df_MTMR,df_MTML], axis=1, sort=False)

    # df.to_csv('/home/npasini1/Autonomous-Camera-Motion/UserStudy/Metrics/'+modality+'_metrics.csv', index = True)
    df.to_csv('/home/npasini1/Desktop/metrics_all/'+modality+'_metrics.csv', index = True)
    print('\n...Saved metrics to CSV file... \n')