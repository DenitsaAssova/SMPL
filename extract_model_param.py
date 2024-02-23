import sys
import os
import numpy as np
import pickle as pkl
import json
import argparse

# from smpl_webuser.serialization import load_model

def get_args():
    """
    Specifies and reads command line arguments

    :return: command line arguments
    """
    parser = argparse.ArgumentParser(description = 'Extract parameters from SMPL model',
                                     formatter_class = argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--gender', '-g', default = 'female', type = str, 
                        help = 'Indicator to specify gender of human model, e.g. female or male, default=None')
    parser.add_argument('--model_dir', '-m', default = './Data/basicmodel_f_lbs_10_207_0_v1.1.0.pkl',type = str, help = 'Directory to model.pkl')
    parser.add_argument('--save_dir', default = './Data/', help = 'Directory to .npz and .json')

    return parser.parse_args()

def main():

    args = get_args()

    if args.gender != None:
        NP_SAVE_FILE = 'smpl_{}.npz'.format(args.gender)
        JSON_SVAE_FILE = 'smpl_{}.json'.format(args.gender)
    else:
        raise SystemError('Please specify gender of the model!\n'
                          'USAGE: \'*f*.pkl\' - female, '
                          '\'*m*.pkl\' - male')

    if not os.path.exists(args.save_dir):
        os.makedirs(args.save_dir)

    np_save_path = os.path.join(args.save_dir, NP_SAVE_FILE)
    json_save_path = os.path.join(args.save_dir, JSON_SVAE_FILE)

    # * Model Data Description * #
    # vertices_template: global vertex locations of template - (6890, 3)
    # face_indices: vertex indices of each face (triangles) - (13776, 3)
    # joint_regressor: joint regressor - (24, 6890)
    # kinematic_tree_table: table of kinematic tree - (2, 24)
    # weights: weights for linear blend skinning - (6890, 24)
    # shape_blend_shapes: shape blend shapes - (6890, 3, 10)
    # pose_blend_shapes: pose blend shapes - (6890, 3, 207)
    #
    # bs_stype: blend skinning style - (default)linear blend skinning
    # bs_type: blend skinning type - (default) linear rotation minimization
    # J: global joint locations of the template mesh - (24, 3)
    # J_regressor_prior: prior joint regressor - (24, 6890)
    # pose_training_info: pose training information - string list with 6 elements.
    # vert_sym_idxs: symmetrical corresponding vertex indices - (6890, )
    # weights_prior: prior weights for linear blend skinning
    
    # model = load_model( '../../models/basicmodel_f_lbs_10_207_0_v1.1.0.pkl' )

    with open(args.model_dir, 'rb') as f:
        model_data = pkl.load(f)
    
    vertices_template = np.array(model_data['v_template'])
    face_indices = np.array(model_data['f'] + 1)  # starts from 1
    weights = np.array(model_data['weights'])
    shape_blend_shapes = np.array(model_data['shapedirs'])
    pose_blend_shapes = np.array(model_data['posedirs'])
    joint_regressor = np.array(model_data['J_regressor'].toarray())
    kinematic_tree = np.array(model_data['kintree_table'])
    joint_template = np.array(model_data['J'])

    model_data_np = {
        'vertices_template': vertices_template,
        'face_indices': face_indices,
        'weights': weights,
        'shape_blend_shapes': shape_blend_shapes,
        'pose_blend_shapes': pose_blend_shapes,
        'joint_regressor': joint_regressor,
        'kinematic_tree': kinematic_tree,
        'joint_template': joint_template
    }

    # Data must be converted to list before storing as json.
    model_data_json = {
        'vertices_template': vertices_template.tolist(),
        'face_indices': face_indices.tolist(),
        'weights': weights.tolist(),
        'shape_blend_shapes': shape_blend_shapes.tolist(),
        'pose_blend_shapes': pose_blend_shapes.tolist(),
        'joint_regressor': joint_regressor.tolist(),
        'kinematic_tree': kinematic_tree.tolist(),
        'joint_template': joint_template.tolist()
    }

    np.savez(np_save_path, **model_data_np)
    
    with open(json_save_path, 'wb+') as f:
        json.dump(model_data_json, f, indent=4, sort_keys=True)

    print('Save SMPL Model to: ', os.path.abspath(args.save_dir))


if __name__ == "__main__":
    if sys.version_info[0] != 2:
        raise EnvironmentError('Run this file with Python2!')
    main()
