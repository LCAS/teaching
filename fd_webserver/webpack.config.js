var webpack = require('webpack');
var inProudction = (process.env.NODE_ENV === 'production');
var path = require('path');
var ExtractTextPlugin = require('extract-text-webpack-plugin');
const CleanWebpackPlugin = require('clean-webpack-plugin');
var CopyWebpackPlugin = require('copy-webpack-plugin');
var ManifestPlugin = require('webpack-manifest-plugin');

module.exports = {

    entry: {
        app: [
            './resources/assets/sass/app.scss',
            './resources/assets/js/app.js',
            'jquery'
        ]
    },

    output: {

        path: path.resolve(__dirname, './static'),

        filename: 'js/[chunkhash].js'
    },

    module: {
        rules: [{
            test: /\.s[ac]ss$/,
            use: ExtractTextPlugin.extract({
                use: ['css-loader', 'sass-loader'],
                fallback: 'style-loader'
            })
        }, {
            test: /\.js$/,
            exclude: '/node_modules/'
        }, {
            test: /\.(png|jpe?g|gif)$/,
            loader: 'file-loader',
            options: {
                name: 'images/[hash].[ext]',
                publicPath: '/'
            }
        }, {
            test: /\.(woff2?|ttf|eot|otf|svg)$/,
            loader: 'file-loader',
            options: {
                name: 'fonts/[hash].[ext]',
                publicPath: '/'
            }
        }],
    },

    plugins: [

        new CleanWebpackPlugin(['static/**/*.*']),

        new ExtractTextPlugin("css/[chunkhash].css"),

        new webpack.LoaderOptionsPlugin({
            minimize: inProudction
        }),

        new webpack.ProvidePlugin({
            $: "jquery",
            jQuery: "jquery"
        }),

        new webpack.DefinePlugin({
            "require.specified": "require.resolve"
        }),

        new CopyWebpackPlugin([{
            from: 'resources/assets/pddl/problems/*.pddl',
            to: path.resolve(__dirname, './static/problems'),
            flatten: true
        }, {
            from: 'resources/assets/pddl/domains/*.pddl',
            to: path.resolve(__dirname, './static/domains/'),
            flatten: true
        }, {
            from: 'resources/assets/img/favicon.ico',
            to: path.resolve(__dirname, './static/favicon.ico'),
            flatten: true
        }]),

        new ManifestPlugin({
            publicPath: "fast-downward/static/"
        })
    ]
};
